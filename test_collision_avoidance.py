#!/usr/bin/env python3
"""
Quick test to verify collision avoidance system is working.
This tests the obstacle detection logic in the navigator.
"""
import math
from lab4.navigator import Navigator, NavConfig
from shared.utils.grid_map import load_grid

def test_front_obstacle_detection():
    """Test that robots detect obstacles in front."""
    # Load a simple grid
    grid = load_grid("shared/maps/warehouse_small.txt")
    
    # Create navigator with collision avoidance enabled
    cfg = NavConfig(
        robot_min_distance=1.0,
        human_min_distance=1.0,
        obstacle_stop_distance=0.8,
        obstacle_slowdown_distance=1.2,
        obstacle_front_angle=1.57,  # ~90°
    )
    nav = Navigator(grid, cfg)
    
    # Test 1: No obstacles
    nav.set_dynamic_obstacles([])
    front_dist, avg_dist = nav._get_front_obstacle_dist(5.0, 5.0, 0.0)
    print(f"✓ Test 1 (no obstacles): front_dist={front_dist}, avg_dist={avg_dist}")
    assert front_dist == float("inf"), "Should return inf when no obstacles"
    
    # Test 2: Obstacle directly ahead (in front)
    nav.set_dynamic_obstacles([(7.0, 5.0)])  # 2m ahead (at 0 radians heading)
    front_dist, avg_dist = nav._get_front_obstacle_dist(5.0, 5.0, 0.0)
    print(f"✓ Test 2 (obstacle ahead): front_dist={front_dist:.2f}m, avg_dist={avg_dist:.2f}m")
    assert front_dist < float("inf"), "Should detect obstacle ahead"
    assert front_dist > 1.9 and front_dist < 2.1, "Should be ~2m away"
    
    # Test 3: Obstacle to the side (NOW DETECTED - 360° detection!)
    nav.set_dynamic_obstacles([(5.0, 7.0)])  # 2m to the right
    front_dist, avg_dist = nav._get_front_obstacle_dist(5.0, 5.0, 0.0)
    print(f"✓ Test 3 (obstacle to side): front_dist={front_dist:.2f}m (360° detection)")
    assert front_dist < float("inf"), "Should detect side obstacle with 360° detection"
    assert front_dist > 1.9 and front_dist < 2.1, "Should be ~2m away (side obstacle)"
    
    # Test 4: Should stop when obstacle is too close
    nav.set_dynamic_obstacles([(5.5, 5.0)])  # 0.5m ahead (closer than stop distance)
    should_stop = nav._should_stop_for_obstacle(5.0, 5.0, 0.0)
    print(f"✓ Test 4 (should stop): should_stop={should_stop}")
    assert should_stop is True, "Should stop when obstacle is < 0.8m ahead"
    
    # Test 5: Should not stop when obstacle is far enough
    nav.set_dynamic_obstacles([(6.5, 5.0)])  # 1.5m ahead (beyond stop distance)
    should_stop = nav._should_stop_for_obstacle(5.0, 5.0, 0.0)
    print(f"✓ Test 5 (should not stop): should_stop={should_stop}")
    assert should_stop is False, "Should not stop when obstacle is > 0.8m ahead"
    
    # Test 6: Multiple obstacles - detects closest in front
    nav.set_dynamic_obstacles([
        (6.0, 5.0),   # 1.0m ahead
        (7.0, 5.0),   # 2.0m ahead
        (5.0, 7.0),   # 2.0m to the right (outside cone)
    ])
    front_dist, avg_dist = nav._get_front_obstacle_dist(5.0, 5.0, 0.0)
    print(f"✓ Test 6 (multiple obstacles): front_dist={front_dist:.2f}m, avg_dist={avg_dist:.2f}m")
    assert front_dist > 0.9 and front_dist < 1.1, "Should detect closest obstacle in front"

    print("\n✅ All collision avoidance detection tests passed!")

if __name__ == "__main__":
    test_front_obstacle_detection()
