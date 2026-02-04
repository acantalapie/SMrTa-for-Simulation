from .greedy_eft import GreedyEarliestFinish
from .round_robin import RoundRobinAssigment
from .interleaved_chunked import GreedyChunckedPickDrop

POLICY_REGISTRY = {
    "greedy_earliest_finish" : GreedyEarliestFinish,
    "greedy_interleaved_chunked" : GreedyChunckedPickDrop,
    "round_robin" : RoundRobinAssigment,
}