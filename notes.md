## Motivations for implementing Fast-RRT*
- Generally, no visible issues with Djikstra, able to work well, path produced
  is plenty reasonable and traversable, capable of not only avoiding obstacles
but also the inflated parts that can still be traversed, resulting in paths
with ample clearance.
- However, the main issue with Djikstra is the "Curse of Dimensionality", which
  basically means that runtime worsens exponentially with the number of
dimensions. Grid-based search generally just doesn't perform well for
high-dimensional problems.
- Performance of Djikstra is also highly dependent on the resolution of the
  grid and worsens as the grid becomes finer (i.e. more cells).  Given that the
cost of Djikstra is naively distance based, if we consider an entirely free map
with a far goal, the algorithm would required to cover a huge bulk of the map
before encountering the goal (by radiating outwards).
- RRT/RRT* as sampling-based methods in general, help to alleviate these
  issues.
- The search time of sampling-based methods are less sensitive to the number of
  dimensions (think of rolling a dice vs slow expansion from the start point). 
- When compared the Djikstra, the number of sample nodes that end up being
  considered/observed usually end up being less than that of Djikstra,
especially in large, sparsely populated environments as above. Effectively,
RRT/RRT* is better at exploring/expanding in open spaces compared to Djikstra.
- RRT* helps to drive optimality through explicit "rewiring" that minimizes
  total path length. Fast-RRT* is an improvement to RRT* based on the premise
that optimal paths are also typically close to obstacles (think wrap around
obstacles)
- We note that in our case, given a 2D problem, it was observed that the
  grid-based methods performed faster on average compared to Fast-RRT, likely
due to the collision checking cost invoked as each node was sampled, where we
query the grid over the line segment to check for obstacles. We do note that
the final path produced by Fast-RRT consisted of far fewer points.
- Note that for multi-dimensional issues, instead of grid checking, use
  computational geometry instead and thus does not require a high dimensional
grid, which can be difficult to create at high dimensions.

## Motivations for A*
- Effectively a more directed form of Djikstra. In an open space, Djikstra
  would naively expand radially towards the goal, but with an added heuristic
cost (Euclidean/Manhattan for example), it informs/directs the search in a much
clever manner, avoiding the outwards radial behaviour and a more 'beeline' like
behaviour, making the search converge faster.
- Between A* and Djikstra, will always prefer Djikstra over A*

## Motivations for DWA
- Constant linear velocity in Pure Pursuit, with angular velocities as function
  of curvature and the constant linear velocity.
- Struggles with turns. Unable to rotate on the spot or if given a point close
  to the robot's current heading, will make a big arc! If there is sufficient
clearance for the robot to perform the arc, no issue but otherwise, collision
occurs!
- DWA overcomes this by generating/sampling candidate velocities and choosing
  the one that best fits the given scenario. In this particular case, it is
possible for DWA to make it such that the robot turns on the spot, or moves
with very minimal linear velocity, subject to cost evaluation, producing
smaller arcs that are more natural/intuitive. Dynamic velocity adaptation!
- Both DWA and PURE need tuned lookahead
