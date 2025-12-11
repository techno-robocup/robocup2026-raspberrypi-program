#!/usr/bin/env python3
"""
CPU and power stress test for Raspberry Pi using matrix operations.
Usage: python3 stress_test.py [duration_seconds] [num_workers] [matrix_size]
Default: 60 seconds, all CPU cores, 512x512 matrices
"""

import multiprocessing
import time
import sys
import numpy as np


def cpu_stress_worker(worker_id: int, duration: float,
                      matrix_size: int) -> None:
  """Perform CPU-intensive matrix calculations."""
  end_time = time.time() + duration
  ops = 0

  while time.time() < end_time:
    # Create random matrices
    a = np.random.rand(matrix_size, matrix_size).astype(np.float64)
    b = np.random.rand(matrix_size, matrix_size).astype(np.float64)

    # Matrix multiply (very CPU intensive, uses BLAS/NEON)
    c = np.dot(a, b)

    # Additional operations to keep pressure
    c = np.linalg.inv(c + np.eye(matrix_size))  # Inversion
    _ = np.linalg.svd(c, full_matrices=False)  # SVD decomposition

    ops += 1

  print(f"Worker {worker_id} finished: {ops} iterations")


def main():
  duration = int(sys.argv[1]) if len(sys.argv) > 1 else 60
  num_workers = int(sys.argv[2]) if len(
      sys.argv) > 2 else multiprocessing.cpu_count()
  matrix_size = int(sys.argv[3]) if len(sys.argv) > 3 else 512

  print(
      f"Starting stress test: {num_workers} workers, {matrix_size}x{matrix_size} matrices, {duration}s"
  )
  print("Press Ctrl+C to stop early")

  processes = []
  start_time = time.time()

  try:
    for i in range(num_workers):
      p = multiprocessing.Process(target=cpu_stress_worker,
                                  args=(i, duration, matrix_size))
      p.start()
      processes.append(p)

    for p in processes:
      p.join()

  except KeyboardInterrupt:
    print("\nStopping...")
    for p in processes:
      p.terminate()
      p.join()

  elapsed = time.time() - start_time
  print(f"Stress test completed in {elapsed:.1f} seconds")


if __name__ == "__main__":
  main()
