# Paths – Graph Search and Dynamic Programming

This project implements algorithms for computing the **shortest** and **longest simple paths** in a directed graph. It supports both cyclic and acyclic graphs and models real-world applications such as puzzle solving, circuit timing, and optimization in state transitions.

## 🎯 Objectives

- Work with directed graphs  
- Implement breadth-first and depth-first graph traversal  
- Detect cycles in a graph  
- Use memoized dynamic programming for longest paths in DAGs  
- Explore all simple paths in cyclic graphs  

## 📦 Features

The program reads a directed graph from a file and computes:

- The **shortest path** (by number of edges) between two vertices using BFS  
- The **longest simple path** using either:
  - Memoized DFS (if the graph is acyclic)  
  - Exhaustive DFS traversal (if the graph contains cycles)

## 📁 Input Format

### 📌 Graph File

```
<vertex_count>
<from_vertex> <to_vertex>
<from_vertex> <to_vertex>
...
```

- First line: number of vertices (non-negative integer)
- Each subsequent line: a directed edge from `<from_vertex>` to `<to_vertex>`

### 📌 Command-line Usage

```bash
./Paths graph_file.in -shortest u v -longest x y ...
```

Each query consists of a triple:
- `-shortest` or `-longest`
- A source vertex
- A destination vertex

### 📌 Example Input File (`graph_6.in`)

```
6
0 1
0 2
0 3
2 1
3 2
3 1
4 0
4 5
5 0
5 4
```

### 📌 Example Command

```bash
./Paths graph_6.in -shortest 4 1 -longest 4 1 -shortest 0 1 -longest 0 1
```

### 📌 Output Format

```
-shortest:   4 ~>   1: 2
 -longest:   4 ~>   1: 5
-shortest:   0 ~>   1: 1
 -longest:   0 ~>   1: 3
```

- Path length `-1` is shown if no path exists.
- Alignment and spacing follow strict format guidelines.

## 🧠 Algorithms Used

- **Shortest path**: Breadth-First Search (BFS)  
- **Cycle detection**: Depth-First Search (DFS) with color marking (`PROCESSING`, `DONE`)  
- **Longest path (DAG)**: Top-down memoized DFS  
- **Longest path (cyclic graph)**: DFS traversal of all simple paths  

## ⚙️ Build & Run

Use the provided Makefile:

```bash
make
./Paths <graph_file> -shortest u v -longest x y
```

## ⏱️ Time Complexity

| Operation                    | Time Complexity         |
|-----------------------------|--------------------------|
| `ldigraph_shortest_path`    | O(n + m)                |
| `ldigraph_longest_path`     | O(n + m) in DAG, else exponential |
| `ldigraph_dfs_visit`        | O(n + m)                |
| `ldigraph_bfs`              | O(n + m)                |

(`n` = number of vertices, `m` = number of edges)

---

This project demonstrates real-world graph traversal techniques and their application in solving shortest and longest path problems across complex networks.
