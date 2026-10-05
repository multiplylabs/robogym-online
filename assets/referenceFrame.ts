/** Remove a shared horizontal world origin without changing height or relative motion. */
export function referenceLocalXY(positions: Float32Array, anchor: ArrayLike<number>): Float32Array {
  const out = positions.slice();
  for (let i = 0; i + 2 < out.length; i += 3) {
    out[i] -= anchor[0]; out[i + 1] -= anchor[1];
  }
  return out;
}
