import typing


class Node:
    def __init__(self, position_x: float = 0.0, position_y: float = 0.0, parent: "Node" = None, cost: float = 0.0):
        self.position_x = position_x
        self.position_y = position_y
        self.parent = parent
        self.cost = cost

     
    def update_parent_and_cost(self, new_parent: "Node"): # Learnt that "Node" in quotes is for forward reference
        self.parent = new_parent
        self.cost = new_parent.cost + self.get_connection_cost(new_parent)
    
    def get_connection_cost(self, other_node: "Node") -> float:
        # Euclidean distance
        return ((self.position_x - other_node.position_x) ** 2 + (self.position_y - other_node.position_y) ** 2) ** 0.5
    
    def __eq__(self, value: "Node"):
        return self.position_x == value.position_x and self.position_y == value.position_y

    def __ne__(self, value: "Node"):
        return not self == value
    
    def copy(self) -> "Node":
        # Needed to do this to avoid reference issues
        return Node(self.position_x, self.position_y, self.parent, self.cost)