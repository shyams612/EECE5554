#!/usr/bin/env python3
"""
GPS Raw Data Collector with Tail

Reads raw NMEA sentences from a GPS device and saves them to a file.
Also displays the last N lines in real-time (like 'tail -f').
No ROS2 dependencies - can run on any Python 3 system.

Usage:
    python3 gps_collector.py -i /dev/ttyUSB0 -o data.txt
    python3 gps_collector.py --input /dev/ttyUSB0 --output scenario1.txt --tail 20
"""

import argparse
import time
import sys
from datetime import datetime
from collections import deque


class GPSDataCollector:
    """Collects raw NMEA data from GPS device and saves to file"""
    
    def __init__(self, input_device, output_file, tail_lines=10, verbose=True):
        """
        Initialize the collector.
        
        Args:
            input_device: Path to GPS device (e.g., /dev/ttyUSB0)
            output_file: Path to output file for saving NMEA sentences
            tail_lines: Number of recent lines to display (0 to disable)
            verbose: Print progress messages
        """
        self.input_device = input_device
        self.output_file = output_file
        self.tail_lines = tail_lines
        self.verbose = verbose
        self.line_count = 0
        self.gpgga_count = 0
        self.start_time = None
        self.recent_lines = deque(maxlen=tail_lines) if tail_lines > 0 else None
        
    def log(self, message):
        """Print message if verbose mode is enabled"""
        if self.verbose:
            print(message)
    
    def display_tail(self):
        """Display the tail buffer of recent lines"""
        if self.recent_lines is None or len(self.recent_lines) == 0:
            return
        
        # Clear screen and move cursor to top
        print("\033[2J\033[H", end="")
        
        # Print header
        elapsed = time.time() - self.start_time if self.start_time else 0
        rate = self.line_count / elapsed if elapsed > 0 else 0
        
        print("=" * 80)
        print(f"GPS Data Collector - Tailing {self.output_file}")
        print("=" * 80)
        print(f"Lines: {self.line_count} | GPGGA: {self.gpgga_count} | "
              f"Rate: {rate:.1f} lines/sec | Duration: {elapsed:.1f}s")
        print(f"Showing last {len(self.recent_lines)} lines (Press Ctrl+C to stop)")
        print("-" * 80)
        
        # Print recent lines
        for line in self.recent_lines:
            print(line)
        
        print("-" * 80)
    
    def collect(self):
        """
        Start collecting raw NMEA data.
        Reads from GPS device and writes to output file.
        Press Ctrl+C to stop collection.
        """
        if not self.tail_lines:
            self.log(f"GPS Data Collector")
            self.log(f"==================")
            self.log(f"Input device: {self.input_device}")
            self.log(f"Output file:  {self.output_file}")
            self.log(f"Started at:   {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
            self.log(f"\nPress Ctrl+C to stop collection\n")
        
        self.start_time = time.time()
        
        try:
            with open(self.input_device, 'r') as input_f, \
                 open(self.output_file, 'w') as output_f:
                
                while True:
                    line = input_f.readline()
                    
                    if line:
                        line = line.strip()
                        
                        # Only write non-empty lines
                        if line:
                            output_f.write(line + '\n')
                            output_f.flush()  # Ensure data is written immediately
                            self.line_count += 1
                            
                            # Count GPGGA sentences specifically
                            if line.startswith('$GPGGA'):
                                self.gpgga_count += 1
                            
                            # Add to tail buffer if enabled
                            if self.recent_lines is not None:
                                self.recent_lines.append(line)
                                # Update display every line
                                self.display_tail()
                            elif self.verbose and self.line_count % 100 == 0:
                                # Print progress every 100 lines (old behavior)
                                elapsed = time.time() - self.start_time
                                rate = self.line_count / elapsed if elapsed > 0 else 0
                                self.log(f"Collected {self.line_count} lines "
                                       f"({self.gpgga_count} GPGGA) - "
                                       f"{rate:.1f} lines/sec")
                    else:
                        # No data available, sleep briefly
                        time.sleep(0.01)
                        
        except KeyboardInterrupt:
            self._print_summary()
        except FileNotFoundError:
            print(f"\nError: Could not open input device '{self.input_device}'")
            print("Make sure the GPS device is connected and you have permissions.")
            print("You may need to run: sudo chmod 666 /dev/ttyUSB0")
            sys.exit(1)
        except PermissionError:
            print(f"\nError: Permission denied for '{self.input_device}'")
            print("Try running: sudo chmod 666 /dev/ttyUSB0")
            print("Or add yourself to dialout group: sudo usermod -a -G dialout $USER")
            sys.exit(1)
        except Exception as e:
            print(f"\nError during collection: {e}")
            self._print_summary()
            sys.exit(1)
    
    def _print_summary(self):
        """Print collection summary statistics"""
        elapsed = time.time() - self.start_time if self.start_time else 0
        
        print("\n" + "=" * 50)
        print("Collection Summary")
        print("=" * 50)
        print(f"Total lines collected: {self.line_count}")
        print(f"GPGGA sentences:       {self.gpgga_count}")
        print(f"Duration:              {elapsed:.1f} seconds")
        if elapsed > 0:
            print(f"Average rate:          {self.line_count / elapsed:.1f} lines/sec")
        print(f"Output file:           {self.output_file}")
        print(f"Stopped at:            {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print("=" * 50)


def main():
    parser = argparse.ArgumentParser(
        description="Collect raw NMEA data from GPS device and save to file",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Collect and tail last 15 lines
  python3 gps_collector.py -i /dev/ttyUSB0 -o urban_walk.txt --tail 15
  
  # Collect without tail (classic mode)
  python3 gps_collector.py -i /dev/ttyUSB0 -o data.txt --no-tail
  
  # Collect from virtual serial port (for testing)
  python3 gps_collector.py -i /dev/pts/2 -o test_data.txt
  
  # Quiet mode (no progress messages, no tail)
  python3 gps_collector.py -i /dev/ttyUSB0 -o data.txt --quiet

Note: You may need permissions to access the GPS device.
      Run: sudo chmod 666 /dev/ttyUSB0
      Or:  sudo usermod -a -G dialout $USER (then log out/in)
        """
    )
    
    parser.add_argument(
        "-i", "--input",
        required=True,
        help="Input GPS device (e.g., /dev/ttyUSB0, /dev/ttyACM0, /dev/pts/2)"
    )
    
    parser.add_argument(
        "-o", "--output",
        required=True,
        help="Output file to save raw NMEA data"
    )
    
    parser.add_argument(
        "-t", "--tail",
        type=int,
        default=10,
        help="Number of recent lines to display (default: 10, use 0 to disable)"
    )
    
    parser.add_argument(
        "--no-tail",
        action="store_true",
        help="Disable tail display (same as --tail 0)"
    )
    
    parser.add_argument(
        "-q", "--quiet",
        action="store_true",
        help="Suppress progress messages (quiet mode, implies --no-tail)"
    )
    
    args = parser.parse_args()
    
    # Determine tail lines
    tail_lines = 0 if (args.no_tail or args.quiet) else args.tail
    
    # Create collector and start collection
    collector = GPSDataCollector(
        input_device=args.input,
        output_file=args.output,
        tail_lines=tail_lines,
        verbose=not args.quiet
    )
    
    collector.collect()


if __name__ == "__main__":
    main()