from .greedy_eft import GreedyEarliestFinish
from .round_robin.py import RoundRobinAssigment

POLICY_REGISTRY = {
    "greedy_earliest_finish" : GreedyEarliestFinish,
    "round_robin" : RoundRobinAssigment,
}