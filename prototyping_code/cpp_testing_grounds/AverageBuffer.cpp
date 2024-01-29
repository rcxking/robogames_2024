/*
 * AverageBuffer.cpp
 *
 * Test program to test out averaging values.
 *
 * Bryant Pong
 * 1/28/24
 */
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <vector>

// Next position to update in the vector
size_t next_pos = 0;

// Old average
double old_avg = 0.0;

// Helper function to display the vector's contents
void PrintVector(const std::vector<int>& vec) {
  for (const int & num : vec) {
    std::cout << num << " ";
  }
  std::cout << std::endl;
}

// Helper function to compute the average of the vector
double ComputeAverage1(const std::vector<int>& vec) {
  const clock_t start_t = clock();
  double sum = 0.0;
  for (const int & num : vec) {
    sum += num;
  }
  const double avg = sum / vec.size();
  std::cout << "ComputeAverage1 took: " <<
    static_cast<double>(clock() - start_t) / CLOCKS_PER_SEC << " seconds"
    << std::endl;
  return avg;
}

// Helper function to compute the average of a vector quicker
double ComputeAverage2(const double old_avg, const int dropped_val,
                       const size_t vec_size, const int new_val) {
  const clock_t start_t = clock();
  const double avg = ((vec_size * old_avg) - dropped_val + new_val) / vec_size;
  std::cout << "ComputeAverage2 took: " <<
    static_cast<double>(clock() - start_t) / CLOCKS_PER_SEC << " seconds"
    << std::endl;
  return avg;
}

// Helper function to update the value of the buffer
void InsertValue(const int num, std::vector<int> &vec) {
  const int dropped_val = vec[next_pos];
  vec[next_pos] = num;
  next_pos = (next_pos + 1) % vec.size();

  const double avg1 = ComputeAverage1(vec);
  const double avg2 = ComputeAverage2(old_avg, dropped_val, vec.size(), num);

  std::cout << "ComputeAverage1: " << avg1 << std::endl;
  std::cout << "ComputeAverage2: " << avg2 << std::endl;

  // Update old average
  old_avg = avg2;
}

int main(int argc, char *argv[]) {
  // Ensure a buffer length is provided
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <buffer length>" << std::endl;
    return 1;
  }

  // Buffer to store the average
  const int buf_size = atoi(argv[1]);
  std::cout << "Creating buffer with size: " << buf_size << std::endl;
  std::vector<int> buffer(buf_size, 0);

  std::cout << "Initial vector: " << std::endl;
  PrintVector(buffer);

  for (int i = 1; i < buf_size+10; ++i) {
    InsertValue(i, buffer);
    PrintVector(buffer);
  }

  return 0;
}
