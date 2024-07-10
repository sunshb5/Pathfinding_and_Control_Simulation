import PathPlan.astar as astar
import PathPlan.bidirectional_astar as bid_astar
import PathPlan.dijkstra as dijkstra
import PathPlan.probabilistic_road_map as prm
import PathTrack.increase_pid_controller as inc_pid
import PathTrack.position_pid_controller as pos_pid
import PathTrack.pure_pursuit_controller as pp
import PathTrack.stanley_controller as stanley
from model.parameter import parameters
from utils.path_length import path_length
from metric.eval import PathEvaluator

# Decide the method
planner_method = {
    "Dijkstra": dijkstra,
    "Astar": astar,
    "Bid_Astar": bid_astar,
    "proba_road_map": prm
}

tracker_method = {
    "increase_pid": inc_pid,
    "position_pid": pos_pid,
    "pure_pursuit": pp,
    "stanley": stanley
}

# Get the planner and tracker based on parameters
planner = planner_method.get(parameters["Plan"], None)
tracker = tracker_method.get(parameters["Track"], None)

plan_path = planner.plan()
path_length = path_length(plan_path[0], plan_path[1])
print(f"规划路径长度: {path_length}\n")

track_path = tracker.track(plan_path)
evaluator = PathEvaluator(plan_path, track_path)
evaluator.print_eval_results()
