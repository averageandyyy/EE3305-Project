# Basic RRT Algorithm (from MATLAB)
- [Video Link](https://youtu.be/QR3U1dgc5RE?si=h18k2Nd1afmVPuLD)
- Start by randomly selecting a node in the state space. Connect to
  nearest node in tree.
  - Will only be connected/considered if distance is within some
    maximum distance threshold and there is no obstacle in the
straight line path.
- In their visualization, if the random node is too far, generate a
  closer one at max distance and connected it instead.
- For RRT(star), node selection process is the same. Generate and
  connect to nearest neighbor. Actually difference comes where the
  connection is established, not necessarily nearest node. Check for
  other nodes within search radius and check if can reconnect in a way
  that maintains tree structure and minimize total path length. Will
  actively try to shorten paths.
- RRT slow because sampling (aka abit of luck involved)
- AStar the heuristic causes it to not hug walls when you'd expect it to. will naively plan towards the goal and then make a uturn of sorts
