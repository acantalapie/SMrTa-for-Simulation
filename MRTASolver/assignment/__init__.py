from .greedy_eft import GreedyEarliestFinish
from .round_robin.py import RoundRobinAssigment

POLICY_REGISTRY = {
    "greedy_eft" : GreedyEarliestFinish,
    "round_robin" : RoundRobinAssigment,
}