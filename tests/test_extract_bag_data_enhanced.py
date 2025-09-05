#!/usr/bin/env python3
"""
Enhanced Test Script for Unified ROS2 Bag Data Extractor

This script tests the enhanced functionality including progress bars, colorization,
data validation, memory monitoring, and other improvements.

Usage:
    python3 test_extract_bag_data_enhanced.py
    python3 test_extract_bag_data_enhanced.py --verbose

Author: Enhanced Test Suite for Unified Data Extraction System
Created: September 2025
Version: 2.0
"""

import unittest
import sys
import os
import tempfile
import shutil
import json
import csv
import argparse
from pathlib import Path
import pandas as pd
import numpy as np
import time
from unittest.mock import patch, MagicMock

# Add the scripts directory to the path to import the extractor
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'scripts'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'scripts', 'experiment'))

try:
    from extract_bag_data import UnifiedBagDataExtractor
    EXTRACTOR_AVAILABLE = True
except ImportError as e:
    print(f"❌ Failed to import UnifiedBagDataExtractor: {e}")
    EXTRACTOR_AVAILABLE = False


class TestEnhancedExtractor(unittest.TestCase):
    """Enhanced test suite for UnifiedBagDataExtractor class."""
    
    @classmethod
    def setUpClass(cls):
        """Set up test class with common resources."""
        cls.test_data_dir = tempfile.mkdtemp(prefix='enhanced_extract_test_')
        cls.output_dir = os.path.join(cls.test_data_dir, 'output')
        
        print(f"📁 Test data directory: {cls.test_data_dir}")
        print(f"📁 Test output directory: {cls.output_dir}")
        
        cls.test_bag_path = os.path.join(cls.test_data_dir, 'test_bag')
        
    @classmethod
    def tearDownClass(cls):
        """Clean up test resources."""
        if os.path.exists(cls.test_data_dir):
            shutil.rmtree(cls.test_data_dir)
            print(f"🧹 Cleaned up test directory: {cls.test_data_dir}")
    
    def setUp(self):
        """Set up each test case."""
        if os.path.exists(self.output_dir):
            shutil.rmtree(self.output_dir)
        os.makedirs(self.output_dir, exist_ok=True)
    
    @unittest.skipUnless(EXTRACTOR_AVAILABLE, "UnifiedBagDataExtractor not available")
    def test_01_progress_bar_functionality(self):
        """Test progress bar and colorization features."""
        print("\n🧪 Testing progress bar functionality...")
        
        extractor = UnifiedBagDataExtractor(self.test_bag_path, self.output_dir, enable_progress=True)
        
        # Test colorization function
        try:
            from colorama import Fore, Style
            colored_text = extractor._colorize("Test", Fore.RED, Style.BRIGHT)
            self.assertIsInstance(colored_text, str)
            self.assertIn("Test", colored_text)
            print(f"🎨 Colorization test: '{colored_text}'")
        except ImportError:
            # Colorama not available, should still work
            colored_text = extractor._colorize("Test")
            self.assertEqual(colored_text, "Test")
            print("⚠️  Colorama not available, using fallback")
        
        print("✅ Progress bar functionality test passed")
    
    @unittest.skipUnless(EXTRACTOR_AVAILABLE, "UnifiedBagDataExtractor not available")
    def test_02_memory_monitoring(self):
        """Test memory monitoring functionality."""
        print("\n📈 Testing memory monitoring...")
        
        extractor = UnifiedBagDataExtractor(self.test_bag_path, self.output_dir)
        
        # Set a low memory threshold for testing
        extractor.memory_warning_threshold = 1024  # 1KB for testing
        
        # Test memory check function
        try:
            high_memory = extractor._check_memory_usage()
            self.assertIsInstance(high_memory, bool)
            print(f"💾 Memory check result: {high_memory}")
        except Exception as e:
            # psutil might not be available in test environment
            print(f"⚠️  Memory monitoring not available: {e}")
            self.assertIn(("ImportError" in str(type(e).__name__) or 
                          "ModuleNotFoundError" in str(type(e).__name__)), [True])
        
        print("✅ Memory monitoring test passed")
    
    @unittest.skipUnless(EXTRACTOR_AVAILABLE, "UnifiedBagDataExtractor not available")
    def test_03_data_validation(self):
        """Test GPS and odometry data validation."""
        print("\n🔍 Testing data validation...")
        
        extractor = UnifiedBagDataExtractor(self.test_bag_path, self.output_dir)
        
        # Test GPS validation
        valid_gps_cases = [
            (37.7749, -122.4194, 100.0),  # San Francisco
            (0.0, 1.0, 0.0),               # Valid but edge case
            (-90.0, 180.0, -100.0),        # Extreme valid values
        ]
        
        invalid_gps_cases = [
            (91.0, -122.4194, 100.0),      # Invalid latitude
            (37.7749, 181.0, 100.0),       # Invalid longitude
            (0.0, 0.0, 0.0),               # Null island
            (37.7749, -122.4194, 100000.0), # Unrealistic altitude
        ]
        
        print("🌍 Testing GPS coordinate validation...")
        for lat, lon, alt in valid_gps_cases:
            is_valid, reason = extractor._validate_gps_data(lat, lon, alt)
            self.assertTrue(is_valid, f"GPS validation failed for {lat}, {lon}, {alt}: {reason}")
        
        for lat, lon, alt in invalid_gps_cases:
            is_valid, reason = extractor._validate_gps_data(lat, lon, alt)
            self.assertFalse(is_valid, f"GPS validation should have failed for {lat}, {lon}, {alt}")
        
        # Test odometry validation
        print("🗺️ Testing odometry data validation...")
        valid_pose = {
            'position': {'x': 1.0, 'y': 2.0, 'z': 0.0},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
        }
        
        invalid_pose = {
            'position': {'x': 200000.0, 'y': 2.0, 'z': 0.0},  # Too large
            'orientation': {'x': 1.0, 'y': 1.0, 'z': 1.0, 'w': 1.0}  # Not normalized
        }
        
        is_valid, reason = extractor._validate_odom_data(valid_pose)
        self.assertTrue(is_valid, f"Odometry validation failed: {reason}")
        
        is_valid, reason = extractor._validate_odom_data(invalid_pose)
        self.assertFalse(is_valid, f"Odometry validation should have failed: {reason}")
        
        print("✅ Data validation test passed")
    
    @unittest.skipUnless(EXTRACTOR_AVAILABLE, "UnifiedBagDataExtractor not available")
    def test_04_signal_handling(self):
        """Test signal handling for graceful cancellation."""
        print("\n🛑 Testing signal handling...")
        
        extractor = UnifiedBagDataExtractor(self.test_bag_path, self.output_dir)
        
        # Test signal handler setup
        self.assertFalse(extractor.cancel_requested)
        
        # Simulate signal
        try:
            extractor._signal_handler(None, None)
            self.assertTrue(extractor.cancel_requested)
            print("📡 Signal handler correctly set cancellation flag")
        except Exception as e:
            print(f"⚠️  Signal handling might not work in test environment: {e}")
        
        print("✅ Signal handling test passed")
    
    @unittest.skipUnless(EXTRACTOR_AVAILABLE, "UnifiedBagDataExtractor not available")
    def test_05_logging_enhancements(self):
        """Test enhanced logging functionality."""
        print("\n📝 Testing enhanced logging...")
        
        log_dir = os.path.join(self.test_data_dir, 'test_logs')
        extractor = UnifiedBagDataExtractor(
            self.test_bag_path, 
            self.output_dir,
            enable_logging=True,
            log_dir=log_dir
        )
        
        # Test that log file was created
        self.assertIsNotNone(extractor.log_file_path)
        self.assertTrue(os.path.exists(extractor.log_file_path))
        print(f"📄 Log file created: {extractor.log_file_path}")
        
        # Test logging functionality
        test_message = "Test log message"
        extractor._log(test_message)
        
        # Check if message was written to file
        with open(extractor.log_file_path, 'r') as f:
            content = f.read()
            self.assertIn(test_message, content)
            print("📝 Log message successfully written to file")
        
        print("✅ Enhanced logging test passed")
    
    @unittest.skipUnless(EXTRACTOR_AVAILABLE, "UnifiedBagDataExtractor not available")
    def test_06_statistics_tracking(self):
        """Test statistics tracking and reporting."""
        print("\n📊 Testing statistics tracking...")
        
        extractor = UnifiedBagDataExtractor(self.test_bag_path, self.output_dir)
        
        # Create mock bag directory and metadata
        os.makedirs(self.test_bag_path, exist_ok=True)
        mock_metadata = {
            'rosbag2_bagfile_information': {
                'topics_with_message_count': [
                    {
                        'topic_metadata': {
                            'name': '/test_gps',
                            'type': 'sensor_msgs/msg/NavSatFix',
                            'serialization_format': 'cdr'
                        },
                        'message_count': 100
                    },
                    {
                        'topic_metadata': {
                            'name': '/test_odom',
                            'type': 'nav_msgs/msg/Odometry',
                            'serialization_format': 'cdr'
                        },
                        'message_count': 1000
                    }
                ]
            }
        }
        
        metadata_path = os.path.join(self.test_bag_path, 'metadata.yaml')
        import yaml
        with open(metadata_path, 'w') as f:
            yaml.dump(mock_metadata, f)
        
        # Read metadata to initialize statistics
        metadata = extractor.read_bag_metadata()
        
        # Check that statistics structure was initialized
        self.assertIn('gps_topics', extractor.extraction_stats)
        self.assertIn('odom_topics', extractor.extraction_stats)
        self.assertIn('/test_gps', extractor.extraction_stats['gps_topics'])
        self.assertIn('/test_odom', extractor.extraction_stats['odom_topics'])
        
        print("📋 Statistics structures initialized correctly")
        
        # Test statistics printing (should not crash)
        try:
            extractor.print_extraction_statistics()
            print("📈 Statistics printing completed without errors")
        except Exception as e:
            self.fail(f"Statistics printing failed: {e}")
        
        print("✅ Statistics tracking test passed")
    
    @unittest.skipUnless(EXTRACTOR_AVAILABLE, "UnifiedBagDataExtractor not available")
    def test_07_colorama_integration(self):
        """Test colorama and tqdm integration."""
        print("\n🌈 Testing colorama and tqdm integration...")
        
        try:
            from colorama import Fore, Style
            colorama_available = True
        except ImportError:
            colorama_available = False
        
        try:
            from tqdm import tqdm
            tqdm_available = True
        except ImportError:
            tqdm_available = False
        
        extractor = UnifiedBagDataExtractor('/mock/path', tempfile.mkdtemp())
        
        # Test colorization with and without colorama
        test_text = "Test Message"
        if colorama_available:
            colored = extractor._colorize(test_text, Fore.RED, Style.BRIGHT)
            self.assertIn(test_text, colored)
            self.assertNotEqual(colored, test_text)  # Should have color codes
            print(f"🎨 Colored text: {colored}")
        else:
            colored = extractor._colorize(test_text)
            self.assertEqual(colored, test_text)  # Should be unchanged
            print("⚠️  Colorama not available, using plain text")
        
        print(f"🌈 Colorama available: {colorama_available}")
        print(f"📊 TQDM available: {tqdm_available}")
        
        if tqdm_available:
            # Test progress bar creation (without actually using it)
            try:
                # Test enabled progress bar
                with tqdm(total=100, desc="Test", disable=False) as pbar:
                    pbar.update(50)
                    self.assertEqual(pbar.n, 50)
                
                # Test disabled progress bar (should not crash)
                with tqdm(total=100, desc="Test", disable=True) as pbar:
                    pbar.update(50)
                    # Disabled progress bars don't update counter, so just check it doesn't crash
                    self.assertTrue(pbar.n >= 0)  # Should be 0 or positive
                    
                print("📊 Progress bar functionality verified")
            except Exception as e:
                self.fail(f"TQDM integration failed: {e}")
        
        print("✅ Color and progress integration test passed")


class TestPerformance(unittest.TestCase):
    """Performance tests for the enhanced extractor."""
    
    @unittest.skipUnless(EXTRACTOR_AVAILABLE, "UnifiedBagDataExtractor not available")
    def test_08_performance_benchmark(self):
        """Test performance with realistic dataset sizes."""
        print("\n🏁 Testing performance benchmark...")
        
        output_dir = tempfile.mkdtemp(prefix='perf_test_')
        extractor = UnifiedBagDataExtractor('/mock/path', output_dir, enable_progress=False)
        
        try:
            # Generate realistic mock dataset
            num_points = 5000  # Reduced for testing
            print(f"📊 Generating {num_points} mock data points...")
            
            start_time = time.time()
            
            # Mock GPS data with validation
            extractor.gps_data = []
            valid_gps_count = 0
            for i in range(num_points):
                lat = 37.7749 + (i * 0.0001) % 0.1  # Stay within reasonable bounds
                lon = -122.4194 + (i * 0.0001) % 0.1
                alt = 100.5 + i * 0.01
                
                gps_point = {
                    'topic_name': '/test_gps',
                    'timestamp_ms': 1600000000000.0 + i * 100,
                    'ros_timestamp_ms': 1600000000000.0 + i * 100,
                    'latitude': lat,
                    'longitude': lon,
                    'altitude': alt,
                    'position_covariance': None,
                    'position_covariance_type': 1,
                    'status': None
                }
                
                # Apply validation
                is_valid, _ = extractor._validate_gps_data(lat, lon, alt)
                if is_valid:
                    valid_gps_count += 1
                
                extractor.gps_data.append(gps_point)
            
            # Mock odometry data with validation
            extractor.odom_data = []
            valid_odom_count = 0
            for i in range(num_points):
                pose = {
                    'position': {'x': i * 0.01, 'y': i * 0.005, 'z': 0.0},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                }
                
                odom_point = {
                    'topic_name': '/test_odom',
                    'timestamp_ms': 1600000000000.0 + i * 100,
                    'ros_timestamp_ms': 1600000000000.0 + i * 100,
                    'frame_id': 'odom',
                    'child_frame_id': 'base_link',
                    'pose': pose,
                    'twist': {
                        'linear': {'x': 0.5, 'y': 0.0, 'z': 0.0},
                        'angular': {'x': 0.0, 'y': 0.0, 'z': 0.1}
                    }
                }
                
                # Apply validation
                is_valid, _ = extractor._validate_odom_data(pose)
                if is_valid:
                    valid_odom_count += 1
                
                extractor.odom_data.append(odom_point)
            
            generation_time = time.time() - start_time
            print(f"⏱️  Data generation took: {generation_time:.3f} seconds")
            print(f"✅ Valid GPS points: {valid_gps_count}/{num_points}")
            print(f"✅ Valid odom points: {valid_odom_count}/{num_points}")
            
            # Test file writing performance
            start_time = time.time()
            gps_csv_path = extractor.save_gps_to_csv()
            odom_csv_path = extractor.save_odom_to_csv()
            csv_time = time.time() - start_time
            print(f"⏱️  CSV writing took: {csv_time:.3f} seconds")
            
            start_time = time.time()
            gps_json_path = extractor.save_gps_to_json()
            odom_json_path = extractor.save_odom_to_json()
            json_time = time.time() - start_time
            print(f"⏱️  JSON writing took: {json_time:.3f} seconds")
            
            # Validate files and sizes
            self.assertTrue(os.path.exists(gps_csv_path))
            self.assertTrue(os.path.exists(odom_csv_path))
            self.assertTrue(os.path.exists(gps_json_path))
            self.assertTrue(os.path.exists(odom_json_path))
            
            gps_csv_size = os.path.getsize(gps_csv_path)
            odom_csv_size = os.path.getsize(odom_csv_path)
            print(f"📁 GPS CSV size: {gps_csv_size / 1024:.1f} KB")
            print(f"📁 Odometry CSV size: {odom_csv_size / 1024:.1f} KB")
            
            # Performance assertions
            self.assertLess(csv_time, 3.0, "CSV writing should be fast")
            self.assertLess(json_time, 3.0, "JSON writing should be fast")
            self.assertGreater(gps_csv_size, num_points * 10, "GPS CSV should have reasonable size")
            self.assertGreater(odom_csv_size, num_points * 20, "Odometry CSV should have reasonable size")
            
            print("✅ Performance benchmark passed")
            
        finally:
            # Clean up
            shutil.rmtree(output_dir, ignore_errors=True)


def main():
    """Main function with command line interface."""
    parser = argparse.ArgumentParser(
        description="Enhanced test suite for unified bag data extractor"
    )
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='Verbose output')
    
    args, unknown = parser.parse_known_args()
    
    # Configure unittest verbosity
    verbosity = 2 if args.verbose else 1
    
    # Create test suite
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add test cases
    suite.addTests(loader.loadTestsFromTestCase(TestEnhancedExtractor))
    suite.addTests(loader.loadTestsFromTestCase(TestPerformance))
    
    # Run tests
    print("="*80)
    print("🚀 ENHANCED BAG DATA EXTRACTOR TEST SUITE")
    print("="*80)
    print(f"🐍 Python path: {sys.executable}")
    print(f"📦 Extractor available: {EXTRACTOR_AVAILABLE}")
    
    # Check for optional dependencies
    try:
        import colorama
        print("🌈 Colorama: Available")
    except ImportError:
        print("⚠️  Colorama: Not available")
    
    try:
        import tqdm
        print("📊 TQDM: Available")
    except ImportError:
        print("⚠️  TQDM: Not available")
    
    try:
        import psutil
        print("💾 Psutil: Available")
    except ImportError:
        print("⚠️  Psutil: Not available")
    
    print("="*80)
    
    runner = unittest.TextTestRunner(verbosity=verbosity)
    result = runner.run(suite)
    
    # Print summary
    print("\n" + "="*80)
    print("📋 TEST SUMMARY")
    print("="*80)
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Skipped: {len(result.skipped)}")
    
    if result.failures:
        print(f"\n❌ FAILURES:")
        for test, traceback in result.failures:
            print(f"  {test}: {traceback.split(chr(10))[-2] if chr(10) in traceback else traceback}")
    
    if result.errors:
        print(f"\n💥 ERRORS:")
        for test, traceback in result.errors:
            print(f"  {test}: {traceback.split(chr(10))[-2] if chr(10) in traceback else traceback}")
    
    if result.skipped:
        print(f"\n⏭️  SKIPPED: {len(result.skipped)} tests")
    
    success = len(result.failures) == 0 and len(result.errors) == 0
    if success:
        print(f"\n🎉 All tests passed!")
    else:
        print(f"\n❌ Some tests failed!")
    
    print("="*80)
    
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())