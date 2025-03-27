

def create_room(x, y):
    f = open(f"data/nx_graph/GRID_{x}x{y}_FREE.graph", 'w')
    
    f.write(f'{x} {y}\n')
    nodes = []
    for x_i in range(x):
        for y_i in range(y):
            nodes.append(f"({x_i}, {y_i})")
            
    f.writelines('#'.join([str(node) for node in nodes]) + '\n')
    i = 0
    for n1 in nodes:
        for n2 in nodes[i:]:
            if is_neighbor(n1, n2):
                f.writelines('#'.join([str(n1), str(n2), str(1)])+'\n')
        i += 1
    f.close()

def is_neighbor(n1, n2):
    x1 = int(n1.split(", ")[0][1:])
    y1 = int(n1.split(", ")[1][:-1])
    x2 = int(n2.split(", ")[0][1:])
    y2 = int(n2.split(", ")[1][:-1])
    
    return (((x1 == x2 + 1 or x1 == x2 - 1) and (y1 == y2)) or \
        ((y1 == y2 + 1 or y1 == y2 - 1) and (x1 == x2)))




X, Y = 2, 2
create_room(X, Y)