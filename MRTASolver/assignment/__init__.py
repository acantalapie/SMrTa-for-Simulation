from .greedy_eft import GreedyEarliestFinish
from .round_robin import RoundRobinAssigment

POLICY_REGISTRY = {
    "greedy_earliest_finish" : GreedyEarliestFinish,
    "round_robin" : RoundRobinAssigment,
}