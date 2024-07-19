## Problem of Overlapping in Point Clouds during Epochs

**Premise:**
The problem we need to address concerns the increase in the size of point clouds as epochs increase. Each epoch merges two point clouds, producing a new, larger point cloud. This leads to a decrease in the percentage of overlap between the source and the target after the ICP (Iterative Closest Point) process.

### Mathematical Model

**Definitions:**

- **L**: width of the Field of View (FoV) in meters, i.e., the width of the point cloud.
- **A**: displacement in meters, produced by the scooter's movement. If the scooter moves at 4 km/h and captures data at 30 FPS, it produces an advance of about 4 cm per frame.
- **E**: epoch, starting from zero and increasing.

### Separation Function

The function that calculates the percentage of separation between source and target is given by:

\[ S(E) = \frac{L - A}{L + E \cdot A} \]

This function accounts for the fact that as epochs increase, the size of the source and target increases, while the absolute overlap remains constant (\( L - A \)).

### Displacement Calculation

Assuming the scooter moves at 4 km/h, this equals \( \frac{4000 \text{ m}}{3600 \text{ s}} = \frac{10}{9} \text{ m/s} \). If it captures 30 frames per second, each frame corresponds to a movement of:

\[ \frac{10/9 \text{ m/s}}{30 \text{ fps}} = \frac{10}{270} \text{ m} \approx 0.037 \text{ m} \approx 4 \text{ cm} \]

### Overlap Considerations

As epochs progress, the size of the point clouds grows, but the absolute overlap remains constant. This leads to a decrease in the percentage of overlap, as shown by the function \( S(E) \).

### Algorithm Modification

To avoid the decrease in percentage overlap, we can modify the algorithm so that, in each epoch, a point cloud is maintained both in the subsequent and the previous fusion. This creates greater overlap as epochs progress.

**Implementation of the Modification:**
1. Each epoch does not simply merge point clouds in pairs but includes redundant points from the previous and next clouds.
2. This allows for an overlap that grows with the epochs, preventing the percentage overlap from drastically decreasing.

### Conclusion

By implementing these modifications, we can maintain greater overlap between point clouds, improving the consistency and quality of the fusion in successive epochs.

---

## Modified Point Cloud Fusion Algorithm to Increase Overlap

To address the problem of decreasing percentage overlap between source and target as epochs increase, we propose a modification to the point cloud fusion algorithm. The idea is to add redundant points from neighboring point clouds to increase the overlap.

### Steps of the Modified Algorithm

1. **Initial Fusion:**
   - Take point clouds \( P_i \) and \( P_{i-1} \) and fuse them to obtain \( P_{i,i-1} \).

2. **Addition of Extra Point Clouds:**
   - After fusing \( P_i \) and \( P_{i-1} \), add points from neighboring point clouds \( P_{i+1} \) and \( P_{i-2} \) to increase the overlap.
   - Create a new point cloud \( P_{i,i-1}' \) that includes points from \( P_{i+1} \) and \( P_{i-2} \).

### Practical Example

Suppose we have point clouds \( P_1, P_2, P_3, \ldots, P_n \). Here's how to proceed:

1. **Epoch n:**
   - Fuse \( P_i \) and \( P_{i-1} \) to obtain \( P_{i,i-1} \).
   - Add points from \( P_{i+1} \) and \( P_{i-2} \) to \( P_{i,i-1} \) to obtain \( P_{i,i-1}' \).

### Formalization of Addition

Define the fusion \( F \) and the addition of points as follows:

\[ P_{i,i-1} = F(P_i, P_{i-1}) \]
\[ P_{i,i-1}' = P_{i,i-1} \cup \{P_{i+1}, P_{i-2}\} \]

### Implementation of the Modification

1. **Fusion:**
   - Implement the fusion function \( F(P_i, P_{i-1}) \) that combines two point clouds.

2. **Addition of Points:**
   - Implement the addition function \( A(P_{i,i-1}, P_{i+1}, P_{i-2}) \) that adds points from point clouds \( P_{i+1} \) and \( P_{i-2} \) to \( P_{i,i-1} \).

### Modified Algorithm