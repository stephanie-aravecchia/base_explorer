This ros package implements basic exploration policies:
* Random Frontier exploration
* Closest Frontier exploration
* Random Free exploration

From an occupancy grid, the node publishes a goal, sampled on the frontier, either random or closest, or sampled randomly in free space.

> "Next-Best-View selection from observation viewpoint statistics", Stéphanie Aravecchia, Antoine Richard, Marianne Clausel, Cédric Pradalier, 2023.