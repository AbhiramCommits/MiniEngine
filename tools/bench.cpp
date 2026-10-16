// minieng_bench: headless benchmark harness.
//
// Scenarios:
//   (a) 500 / 2000 / 5000 dynamic bodies in a box, N ticks each, comparing the
//       brute-force O(n^2) broadphase against sweep-and-prune.
//   (b) 100k raycasts and 100k AABB sweeps against the 5000-body scene.
//   (c) determinism: same seed twice must hash equal, a different seed must not.
//
// Fixed seeds, warm-up ticks discarded, human table on stdout and machine
// readable results/bench.json. No window, no GLFW.

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "minieng/loop.hpp"
#include "minieng/physics.hpp"
#include "minieng/profiler.hpp"
#include "minieng/rng.hpp"
#include "minieng/scene.hpp"
#include "minieng/scene_loader.hpp"
#include "minieng/world.hpp"

using namespace minieng;

namespace {

constexpr float kDt = 1.0f / 60.0f;

// ---------------------------------------------------------------------------
// Scene building
// ---------------------------------------------------------------------------

struct BenchScene {
    World world;
    Scene scene;
    PhysicsWorld physics{world, scene};

    BenchScene() {
        PhysicsConfig& cfg = physics.config();
        // Zero gravity keeps the bodies spread through the box so the numbers
        // isolate broadphase + query cost instead of solver convergence.
        cfg.gravity = Vec3::zero();
        cfg.linear_damping = 0.0f;
        cfg.angular_damping = 0.0f;
        cfg.solver_iterations = 1;
    }
};

EntityId add_box(World& world, Scene& scene, const Vec3& position, const Vec3& half, bool is_static,
                 float restitution, float friction) {
    const EntityId id = world.create_entity();
    const int node = scene.create_node(-1, id);
    Transform t;
    t.position = position;
    scene.set_local(node, t);
    world.add_transform_ref(id, node);

    Collider c;
    c.shape = Collider::AABB;
    c.half_extents = half;
    world.add_collider(id, c);

    RigidBody b;
    b.is_static = is_static;
    b.mass = is_static ? 0.0f : 1.0f;
    b.restitution = restitution;
    b.friction = friction;
    world.add_rigid_body(id, b);
    return id;
}

void build_box_scene(BenchScene& sim, int bodies, std::uint64_t seed, bool use_sap) {
    sim.physics.set_use_sap(use_sap);

    const float box_half = 30.0f;
    // Floor and four walls (an open-top box).
    add_box(sim.world, sim.scene, Vec3(0.0f, -0.5f, 0.0f), Vec3(box_half, 0.5f, box_half), true,
            0.1f, 0.5f);
    add_box(sim.world, sim.scene, Vec3(-box_half - 0.5f, box_half, 0.0f),
            Vec3(0.5f, box_half, box_half), true, 0.1f, 0.5f);
    add_box(sim.world, sim.scene, Vec3(box_half + 0.5f, box_half, 0.0f),
            Vec3(0.5f, box_half, box_half), true, 0.1f, 0.5f);
    add_box(sim.world, sim.scene, Vec3(0.0f, box_half, -box_half - 0.5f),
            Vec3(box_half, box_half, 0.5f), true, 0.1f, 0.5f);
    add_box(sim.world, sim.scene, Vec3(0.0f, box_half, box_half + 0.5f),
            Vec3(box_half, box_half, 0.5f), true, 0.1f, 0.5f);

    Rng rng(seed);
    for (int i = 0; i < bodies; ++i) {
        const Vec3 position(rng.range(-box_half + 1.0f, box_half - 1.0f),
                            rng.range(0.5f, box_half * 2.0f - 1.0f),
                            rng.range(-box_half + 1.0f, box_half - 1.0f));
        const Vec3 half(rng.range(0.35f, 0.65f), rng.range(0.35f, 0.65f), rng.range(0.35f, 0.65f));
        const EntityId id = add_box(sim.world, sim.scene, position, half, false, 0.2f, 0.4f);
        RigidBody* body = sim.world.get_rigid_body(id);
        body->velocity =
            Vec3(rng.range(-0.4f, 0.4f), rng.range(-0.4f, 0.4f), rng.range(-0.4f, 0.4f));
    }

    sim.scene.update_world_transforms();
    sim.physics.build_broadphase();
}

// ---------------------------------------------------------------------------
// Timing helpers
// ---------------------------------------------------------------------------

double now_ms() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double, std::milli>(clock::now().time_since_epoch()).count();
}

struct TickRun {
    FrameStats stats;
    double mean_pairs = 0.0;
    double mean_contacts = 0.0;
    std::vector<std::pair<std::string, FrameStats>> systems;
};

TickRun run_ticks(BenchScene& sim, int total_ticks, int warmup_ticks) {
    Profiler& profiler = Profiler::global();
    profiler.reset();

    std::vector<double> tick_ms;
    tick_ms.reserve(static_cast<std::size_t>(std::max(0, total_ticks - warmup_ticks)));

    double pair_sum = 0.0;
    double contact_sum = 0.0;
    int measured = 0;

    for (int tick = 0; tick < total_ticks; ++tick) {
        const double t0 = now_ms();
        profiler.begin_frame();

        {
            MINIENG_PROFILE("integrate");
            sim.physics.integrate(kDt);
        }

        std::vector<Pair> pairs;
        {
            MINIENG_PROFILE("broadphase");
            sim.physics.build_broadphase();
            pairs = sim.physics.broadphase_pairs();
        }

        std::vector<Contact> contacts;
        {
            MINIENG_PROFILE("narrowphase");
            contacts = sim.physics.narrowphase(pairs);
        }

        {
            MINIENG_PROFILE("resolve");
            sim.physics.resolve_contacts(contacts, kDt);
        }

        {
            MINIENG_PROFILE("transforms");
            sim.scene.update_world_transforms();
        }

        profiler.end_frame();
        const double t1 = now_ms();

        if (tick >= warmup_ticks) {
            tick_ms.push_back(t1 - t0);
            pair_sum += static_cast<double>(pairs.size());
            contact_sum += static_cast<double>(contacts.size());
            ++measured;
        }
    }

    TickRun run;
    run.stats = compute_stats(tick_ms);
    if (measured > 0) {
        run.mean_pairs = pair_sum / measured;
        run.mean_contacts = contact_sum / measured;
    }
    run.systems = profiler.all_stats();
    return run;
}

struct QueryResult {
    int count = 0;
    double ray_ms = 0.0;
    int ray_hits = 0;
    double sweep_ms = 0.0;
    int sweep_hits = 0;
};

QueryResult measure_queries(PhysicsWorld& physics, int count, std::uint64_t seed) {
    QueryResult result;
    result.count = count;
    physics.build_broadphase();

    Rng rng(seed);
    const double t0 = now_ms();
    for (int i = 0; i < count; ++i) {
        const Vec3 origin(rng.range(-25.0f, 25.0f), rng.range(10.0f, 55.0f),
                          rng.range(-25.0f, 25.0f));
        const Vec3 dir(rng.range(-1.0f, 1.0f), -1.0f, rng.range(-1.0f, 1.0f));
        if (physics.raycast(origin, dir, 200.0f).hit) {
            ++result.ray_hits;
        }
    }
    result.ray_ms = now_ms() - t0;

    const Vec3 half(0.4f, 0.4f, 0.4f);
    const double t1 = now_ms();
    for (int i = 0; i < count; ++i) {
        const Vec3 from(rng.range(-25.0f, 25.0f), rng.range(10.0f, 55.0f),
                        rng.range(-25.0f, 25.0f));
        const Vec3 to = from + Vec3(rng.range(-6.0f, 6.0f), -20.0f, rng.range(-6.0f, 6.0f));
        Aabb box;
        box.min = from - half;
        box.max = from + half;
        if (physics.sweep_aabb(box, from, to).hit) {
            ++result.sweep_hits;
        }
    }
    result.sweep_ms = now_ms() - t1;
    return result;
}

std::string machine_description() {
#if defined(__APPLE__)
    return "macOS " + std::string(sizeof(void*) == 8 ? "64-bit" : "32-bit");
#elif defined(__linux__)
    return "Linux 64-bit";
#else
    return "unknown";
#endif
}

// Benchmark a scene file supplied with --scene: one timed tick run plus query
// throughput. Writes the same JSON shape under a "custom_scene" key.
int run_scene_bench(const std::string& path, int ticks, int warmup, int queries,
                    const std::string& json_path) {
    BenchScene sim;
    const SceneDescription desc = load_scene_from_file(path, sim.world, sim.scene);
    if (!desc.ok) {
        std::fprintf(stderr, "could not load scene '%s': %s\n", path.c_str(), desc.error.c_str());
        return 1;
    }
    sim.physics.config().gravity = desc.gravity;
    sim.physics.build_broadphase();

    std::printf("=== custom scene: %s ===\n", desc.name.empty() ? path.c_str() : desc.name.c_str());
    std::printf("file=%s entities=%zu gravity=(%.2f, %.2f, %.2f)\n", path.c_str(),
                sim.world.entity_count(), static_cast<double>(desc.gravity.x),
                static_cast<double>(desc.gravity.y), static_cast<double>(desc.gravity.z));

    const TickRun run = run_ticks(sim, ticks, warmup);
    std::printf("%8s  %-11s %10s %10s %10s %12s %12s\n", "bodies", "broadphase", "mean_ms",
                "p95_ms", "max_ms", "pairs", "contacts");
    std::printf("%8zu  %-11s %10.3f %10.3f %10.3f %12.1f %12.1f\n", sim.world.entity_count(), "sap",
                run.stats.mean_ms, run.stats.p95_ms, run.stats.max_ms, run.mean_pairs,
                run.mean_contacts);

    std::printf("\n%-14s %10s %10s\n", "system", "mean_ms", "share");
    for (const auto& entry : run.systems) {
        if (entry.second.samples == 0) {
            continue;
        }
        const double share =
            run.stats.mean_ms > 1.0e-9 ? 100.0 * entry.second.mean_ms / run.stats.mean_ms : 0.0;
        std::printf("%-14s %10.3f %9.1f%%\n", entry.first.c_str(), entry.second.mean_ms, share);
    }

    const QueryResult query = measure_queries(sim.physics, queries, 4242u);
    const double ray_per_sec = query.ray_ms > 1.0e-9 ? query.count * 1000.0 / query.ray_ms : 0.0;
    const double sweep_per_sec =
        query.sweep_ms > 1.0e-9 ? query.count * 1000.0 / query.sweep_ms : 0.0;
    std::printf("\nraycasts   %7d  %10.3f ms  %14.0f /s\n", query.count, query.ray_ms, ray_per_sec);
    std::printf("sweeps     %7d  %10.3f ms  %14.0f /s\n", query.count, query.sweep_ms,
                sweep_per_sec);

    std::ostringstream json;
    json.precision(6);
    json << std::fixed;
    json << "{\n";
    json << "  \"generated_by\": \"minieng_bench\",\n";
    json << "  \"mode\": \"custom_scene\",\n";
    json << "  \"machine\": \"" << machine_description() << "\",\n";
    json << "  \"scene\": \"" << path << "\",\n";
    json << "  \"scene_name\": \"" << desc.name << "\",\n";
    json << "  \"entities\": " << sim.world.entity_count() << ",\n";
    json << "  \"ticks\": " << ticks << ",\n";
    json << "  \"warmup_ticks\": " << warmup << ",\n";
    json << "  \"mean_ms\": " << run.stats.mean_ms << ",\n";
    json << "  \"p95_ms\": " << run.stats.p95_ms << ",\n";
    json << "  \"mean_pairs\": " << run.mean_pairs << ",\n";
    json << "  \"mean_contacts\": " << run.mean_contacts << ",\n";
    json << "  \"raycasts\": " << query.count << ",\n";
    json << "  \"raycasts_per_sec\": " << ray_per_sec << ",\n";
    json << "  \"sweeps\": " << query.count << ",\n";
    json << "  \"sweeps_per_sec\": " << sweep_per_sec << "\n";
    json << "}\n";

    std::error_code ec;
    std::filesystem::create_directories(std::filesystem::path(json_path).parent_path(), ec);
    std::ofstream out(json_path);
    if (!out) {
        std::fprintf(stderr, "could not write %s\n", json_path.c_str());
        return 1;
    }
    out << json.str();
    out.close();
    std::printf("wrote %s\n", json_path.c_str());
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    int ticks = 1800;
    int warmup = 60;
    int ray_queries = 100000;
    std::string json_path = "results/bench.json";
    std::string scene_path;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto next_int = [&](int fallback) {
            return (i + 1 < argc) ? std::atoi(argv[++i]) : fallback;
        };
        if (arg == "--ticks") {
            ticks = next_int(ticks);
        } else if (arg == "--warmup") {
            warmup = next_int(warmup);
        } else if (arg == "--queries") {
            ray_queries = next_int(ray_queries);
        } else if (arg == "--json") {
            if (i + 1 < argc) {
                json_path = argv[++i];
            }
        } else if (arg == "--scene") {
            if (i + 1 < argc) {
                scene_path = argv[++i];
            }
        } else if (arg == "--help" || arg == "-h") {
            std::printf(
                "usage: minieng_bench [--ticks N] [--warmup N] [--queries N] [--json PATH] "
                "[--scene PATH]\n");
            return 0;
        }
    }

    if (!scene_path.empty()) {
        return run_scene_bench(scene_path, ticks, warmup, ray_queries, json_path);
    }

    std::printf("MiniEngine benchmark\n");
    std::printf("  ticks=%d warmup=%d dt=%.6f queries=%d\n\n", ticks, warmup, kDt, ray_queries);

    const std::vector<int> body_counts = {500, 2000, 5000};

    struct RowA {
        int bodies;
        const char* broadphase;
        FrameStats stats;
        double mean_pairs;
        double mean_contacts;
    };
    std::vector<RowA> rows;
    std::vector<std::pair<std::string, FrameStats>> system_rows;
    FrameStats big_stats;

    std::printf("=== (a) broadphase: brute force vs sweep-and-prune ===\n");
    std::printf("%8s  %-11s %10s %10s %10s %12s %12s\n", "bodies", "broadphase", "mean_ms",
                "p95_ms", "max_ms", "pairs", "contacts");

    for (const int bodies : body_counts) {
        for (const bool use_sap : {false, true}) {
            BenchScene sim;
            build_box_scene(sim, bodies, 20261016ull + static_cast<std::uint64_t>(bodies), use_sap);
            const TickRun run = run_ticks(sim, ticks, warmup);

            rows.push_back(RowA{bodies, use_sap ? "sap" : "bruteforce", run.stats, run.mean_pairs,
                                run.mean_contacts});

            std::printf("%8d  %-11s %10.3f %10.3f %10.3f %12.1f %12.1f\n", bodies,
                        use_sap ? "sap" : "bruteforce", run.stats.mean_ms, run.stats.p95_ms,
                        run.stats.max_ms, run.mean_pairs, run.mean_contacts);
            std::fflush(stdout);

            if (bodies == 5000 && use_sap) {
                big_stats = run.stats;
                for (const auto& entry : run.systems) {
                    if (entry.second.samples > 0) {
                        system_rows.push_back(entry);
                    }
                }
            }
        }
    }
    std::printf("\n");

    // Per-system breakdown for the largest sweep-and-prune run.
    std::printf("=== (a) per-system breakdown, 5000 bodies, sap ===\n");
    std::printf("%-14s %10s %10s\n", "system", "mean_ms", "share");
    for (const auto& entry : system_rows) {
        const double share =
            big_stats.mean_ms > 1.0e-9 ? 100.0 * entry.second.mean_ms / big_stats.mean_ms : 0.0;
        std::printf("%-14s %10.3f %9.1f%%\n", entry.first.c_str(), entry.second.mean_ms, share);
    }
    std::printf("%-14s %10.3f %9.1f%%\n", "tick_total", big_stats.mean_ms, 100.0);
    std::printf("\n");

    // A 5000-body scene for the query and determinism scenarios.
    BenchScene big;
    build_box_scene(big, 5000, 20261016ull + 5000ull, true);

    // --- (b) query throughput ---------------------------------------------
    std::printf("=== (b) queries against the 5000-body scene ===\n");
    const QueryResult query = measure_queries(big.physics, ray_queries, 4242u);
    const int ray_hits = query.ray_hits;
    const double ray_ms = query.ray_ms;
    const int sweep_hits = query.sweep_hits;
    const double sweep_ms = query.sweep_ms;
    const double ray_per_sec = ray_ms > 1.0e-9 ? ray_queries * 1000.0 / ray_ms : 0.0;
    const double sweep_per_sec = sweep_ms > 1.0e-9 ? ray_queries * 1000.0 / sweep_ms : 0.0;
    std::printf("raycasts   %7d  %10.3f ms  %14.0f /s  hit_rate=%.2f\n", ray_queries, ray_ms,
                ray_per_sec, static_cast<double>(ray_hits) / ray_queries);
    std::printf("sweeps     %7d  %10.3f ms  %14.0f /s  hit_rate=%.2f\n", ray_queries, sweep_ms,
                sweep_per_sec, static_cast<double>(sweep_hits) / ray_queries);
    std::printf("\n");
    std::fflush(stdout);

    // --- (c) determinism ---------------------------------------------------
    std::printf("=== (c) determinism ===\n");
    const auto build_and_hash = [](std::uint64_t seed, int bodies, int det_ticks) {
        BenchScene sim;
        build_box_scene(sim, bodies, seed, true);
        for (int i = 0; i < det_ticks; ++i) {
            sim.physics.step(kDt);
        }
        return sim.world.hash(sim.scene);
    };
    const std::uint64_t hash_a = build_and_hash(777u, 300, 300);
    const std::uint64_t hash_b = build_and_hash(777u, 300, 300);
    const std::uint64_t hash_c = build_and_hash(778u, 300, 300);
    const bool same_seed_match = (hash_a == hash_b);
    const bool different_seed_differs = (hash_a != hash_c);
    std::printf("same seed, two runs : %s  (0x%016llx)\n", same_seed_match ? "identical" : "DIFFER",
                static_cast<unsigned long long>(hash_a));
    std::printf("different seed      : %s  (0x%016llx)\n",
                different_seed_differs ? "differs" : "SAME (bad)",
                static_cast<unsigned long long>(hash_c));
    std::printf("\n");
    std::fflush(stdout);

    // --- JSON --------------------------------------------------------------
    std::ostringstream json;
    json.precision(6);
    json << std::fixed;
    json << "{\n";
    json << "  \"generated_by\": \"minieng_bench\",\n";
    json << "  \"machine\": \"" << machine_description() << "\",\n";
    json << "  \"dt\": " << kDt << ",\n";
    json << "  \"ticks\": " << ticks << ",\n";
    json << "  \"warmup_ticks\": " << warmup << ",\n";
    json << "  \"body_counts\": [500, 2000, 5000],\n";

    json << "  \"scenario_a\": [\n";
    for (std::size_t i = 0; i < rows.size(); ++i) {
        const RowA& r = rows[i];
        json << "    { \"bodies\": " << r.bodies << ", \"broadphase\": \"" << r.broadphase
             << "\", \"mean_ms\": " << r.stats.mean_ms << ", \"p50_ms\": " << r.stats.p50_ms
             << ", \"p95_ms\": " << r.stats.p95_ms << ", \"max_ms\": " << r.stats.max_ms
             << ", \"mean_pairs\": " << r.mean_pairs << ", \"mean_contacts\": " << r.mean_contacts
             << " }" << (i + 1 < rows.size() ? "," : "") << "\n";
    }
    json << "  ],\n";

    json << "  \"systems_5000_sap\": [\n";
    for (std::size_t i = 0; i < system_rows.size(); ++i) {
        const double share = big_stats.mean_ms > 1.0e-9
                               ? 100.0 * system_rows[i].second.mean_ms / big_stats.mean_ms
                               : 0.0;
        json << "    { \"system\": \"" << system_rows[i].first
             << "\", \"mean_ms\": " << system_rows[i].second.mean_ms
             << ", \"p95_ms\": " << system_rows[i].second.p95_ms << ", \"share_percent\": " << share
             << " }" << (i + 1 < system_rows.size() ? "," : "") << "\n";
    }
    json << "  ],\n";
    json << "  \"tick_total_ms_5000_sap\": " << big_stats.mean_ms << ",\n";

    json << "  \"scenario_b\": {\n";
    json << "    \"bodies\": 5000,\n";
    json << "    \"raycasts\": " << ray_queries << ",\n";
    json << "    \"raycasts_per_sec\": " << (ray_ms > 1.0e-9 ? ray_queries * 1000.0 / ray_ms : 0.0)
         << ",\n";
    json << "    \"raycast_hit_rate\": "
         << (ray_queries > 0 ? static_cast<double>(ray_hits) / ray_queries : 0.0) << ",\n";
    json << "    \"sweeps\": " << ray_queries << ",\n";
    json << "    \"sweeps_per_sec\": "
         << (sweep_ms > 1.0e-9 ? ray_queries * 1000.0 / sweep_ms : 0.0) << ",\n";
    json << "    \"sweep_hit_rate\": "
         << (ray_queries > 0 ? static_cast<double>(sweep_hits) / ray_queries : 0.0) << "\n";
    json << "  },\n";

    json << "  \"scenario_c\": {\n";
    json << "    \"bodies\": 300,\n";
    json << "    \"ticks\": 300,\n";
    json << "    \"same_seed_identical\": " << (same_seed_match ? "true" : "false") << ",\n";
    json << "    \"different_seed_differs\": " << (different_seed_differs ? "true" : "false")
         << ",\n";
    json << "    \"hash_a\": " << static_cast<unsigned long long>(hash_a) << ",\n";
    json << "    \"hash_b\": " << static_cast<unsigned long long>(hash_b) << ",\n";
    json << "    \"hash_c\": " << static_cast<unsigned long long>(hash_c) << "\n";
    json << "  }\n";
    json << "}\n";

    std::error_code ec;
    std::filesystem::create_directories(std::filesystem::path(json_path).parent_path(), ec);
    std::ofstream out(json_path);
    if (!out) {
        std::fprintf(stderr, "could not write %s\n", json_path.c_str());
        return 1;
    }
    out << json.str();
    out.close();
    std::printf("wrote %s\n", json_path.c_str());

    return 0;
}
