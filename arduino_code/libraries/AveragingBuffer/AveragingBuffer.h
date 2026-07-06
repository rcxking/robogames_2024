#ifndef __AVERAGING_BUFFER_H__
#define __AVERAGING_BUFFER_H__

/*
 * AveragingBuffer.h
 *
 * Class definition to implement an averaging buffer.
 */
#include "Arduino.h"

#include <assert.h>
#include <string.h>

class AveragingBuffer {
public:
  /*
   * Constructor.
   *
   * Parameters:
   *   size (size_t): Number of elements in this buffer
   */
  AveragingBuffer(const size_t size) : size_(size), avg_(0.0) {
    // Ensure user specified a nonzero size
    assert(size != 0);

    // Dynamically allocate storage space
    buff_ = new double[size_];
    memset(buff_, 0, sizeof(double) * size_);
  }

  // Destructor
  ~AveragingBuffer() {
    // Delete buffer if allocated
    if (buff_ != nullptr) {
      delete [] buff_;
      buff_ = nullptr;
    }

    size_ = 0;
    avg_  = 0.0;
  }

  /*
   * Adds the specified element to the buffer.  If out of space insert this
   * element in the 0th index.
   *
   * Parameters:
   *   element (double): Next element to insert
   */
  void Insert(const double element) {
    // Current index to insert element
    static size_t idx = 0;

    /*
     * Computes/updates the average of all the elements in this buffer.
     *
     * Let n = # of elements in this buffer and x1, x2, ..., xn be the elements
     * in this buffer.  The average is therefore:
     *
     *           x1 + x2 + ... + xn
     * average = ------------------ -> average * n = x1 + x2 + ... + xn ->
     *                   n
     *
     * Whenever a new element is added, we need to drop the oldest element x1
     * and add in this new element.  Thus we have:
     *
     * old_average * n - x1 + element = x2 + ... + xn + element
     *
     *               x2 + ... + xn + element   old_average * n - x1 + element
     * new_average = ----------------------- = ------------------------------
     *                          n                              n
     */

    // Oldest element to drop
    const double element_to_drop = buff_[idx];

    // Update the running average
    const double new_avg = ((size_ * avg_) - element_to_drop + element) / size_;
    avg_ = new_avg;

    // Add the new element to the buffer and update the index
    buff_[idx] = element;
    idx = (idx + 1) % size_;
  }

  // Accessors
  double GetAverage() const { return avg_; }

private:
  // Max number of elements that can be stored
  size_t size_ = 0;

  // Buffer is used to average wheel velocities
  double *buff_ = nullptr;

  // Stores the running average of all values in this buffer
  double avg_ = 0.0;

  // Disallow copy constructor and assignment operator=
  AveragingBuffer(const AveragingBuffer &) = delete;
  AveragingBuffer& operator=(const AveragingBuffer &) = delete;
};

#endif
