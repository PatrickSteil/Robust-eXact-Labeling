# RXL: sampled pruned hub labeling

C++17 exact distance index for directed, positively weighted graphs. It combines
weighted Pruned Labeling with a SamPG-style sampled path-greedy vertex order.

The implementation follows the paper's SamPG design (sampled shortest-path
trees, robust counter buckets, covered-subtree deletion, pruned replenishment,
and work balancing), including the paper's hash-table memory engineering for
small sampled trees, as well as reverse-index acceleration (Appendix A.2).
