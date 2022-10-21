def _swap(L, i, j):
    L[i], L[j] = L[j], L[i]

def _bottom_up(heap, index):

    root_index = (index - 1) // 2

    if root_index < 0:
        return

    if heap[index].total_cost > heap[root_index].total_cost:
        _swap(heap, index, root_index)

        _bottom_up(heap, root_index)

def _top_down(heap, index):
    child_index = 2 * index + 1

    if child_index >= len(heap):
        return

    if child_index + 1 < len(heap) and heap[child_index].total_cost < heap[child_index + 1].total_cost:
        child_index += 1

    if heap[child_index].total_cost > heap[index].total_cost:
        _swap(heap, child_index, index)
        _top_down(heap, child_index)

class Max_Heap:
    def __init__(self, arr=[]):
        self._heap = []

        if arr is not None:
            for node in arr:
                self.push(node)

    def push(self, node):
        self._heap.append(node)
        _bottom_up(self._heap, len(self) - 1)

    def pop(self):
        if len(self._heap) != 0:
            _swap(self._heap, len(self) - 1, 0)
            root_node = self._heap.pop()
            _top_down(self._heap, 0)

        return root_node

    def __len__(self):
        return len(self._heap)

    def empty(self):
        if(len == 0):
            return True
        return False
def say_hello():
   print( 'Hello, world!' )