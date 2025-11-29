use parry3d::bounding_volume::Aabb;
use parry3d::na::Point3;
use parry3d::partitioning::{Bvh, BvhBuildStrategy}; // nalgebra re-exported

fn main() {
    println!("Setting up Parry3D BVH panic reproduction...");

    let mut aabbs = Vec::new();

    // 1. Create a base point.
    let origin = Point3::origin();

    // 2. Add a 'base' leaf at the origin.
    // Center is (0.5, 0.5, 0.5)
    aabbs.push(Aabb::new(origin, Point3::new(1.0, 1.0, 1.0)));

    // 3. Add a second leaf shifted by the smallest possible positive subnormal float.
    // f32::from_bits(1) is approx 1.4e-45.
    let tiny_offset = f32::from_bits(1);

    // Shift position by tiny_offset on the X axis.
    // New center will be (0.5 + tiny_offset, 0.5, 0.5).
    let p1 = Point3::new(tiny_offset, 0.0, 0.0);
    let p2 = Point3::new(1.0 + tiny_offset, 1.0, 1.0);
    aabbs.push(Aabb::new(p1, p2));

    // 4. Fill with some dummy data to ensure the builder runs in a loop.
    for _ in 2..10 {
        aabbs.push(Aabb::new(origin, Point3::new(1.0, 1.0, 1.0)));
    }

    println!(
        "Constructing BVH with a subnormal extent range (approx {:e})...",
        tiny_offset
    );
    println!("This should trigger: 'index out of bounds: the len is 8 but the index is ...'");

    // CORRECTED CALL:
    // We use `from_leaves` with the default strategy (which uses the binned builder).
    let _bvh = Bvh::from_leaves(BvhBuildStrategy::default(), &aabbs);

    println!("BVH built successfully (Panic failed to trigger).");
}
