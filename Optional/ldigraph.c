#include <stdlib.h>
#include <stdbool.h>

#include "ldigraph.h"

//for cycle detection
static bool check_cycle = false;

struct ldigraph
{
  size_t n;          // the number of vertices
  size_t *list_size; // the size of each adjacency list
  size_t *list_cap;  // the capacity of each adjacency list
  size_t **adj;      // the adjacency lists
};

typedef struct
{
  const ldigraph *g; // the graph that was searched
  int *color; // current status of each vertex (using enum below)
  int *dist; // number of edges on the path that was found to each vertex
             // (not meaningful for DFS)
  int *pred; // predecessor along the path that was found (won't be needed)
  // YOU CAN ADD MORE THINGS HERE!
} ldigraph_search;

enum {LDIGRAPH_UNSEEN, LDIGRAPH_PROCESSING, LDIGRAPH_DONE};

#define LDIGRAPH_ADJ_LIST_INITIAL_CAPACITY 4

// YOU MAY CHANGE THE SIGNATURES OF ANY OF THE FUNCTIONS BELOW AS YOU SEE FIT

/**
 * Returns the result of running breadth-first search on the given
 * graph starting with the given vertex.  When the search arrives
 * at a vertex, its neighbors are considered in the order the
 * corresponding edges were added to the graph.
 * It is the caller's responsibility to destroy to result.
 *
 * @param g a pointer to a directed graph, non-NULL
 * @param from the index of a vertex in the given graph
 * @return the result of the search
 */
static ldigraph_search *ldigraph_bfs(const ldigraph *g, size_t from);


/**
 * Returns the result of running depth-first search on the given
 * graph starting with the given vertex.  When the search arrives
 * at a vertex, its neighbors are considered in the order the
 * corresponding edges were added to the graph.
 * It is the caller's responsibility to destroy to result.
 *
 * @param g a pointer to a directed graph, non-NULL
 * @param from the index of a vertex in the given graph
 * @return the result of the search
 */
static ldigraph_search *ldigraph_dfs(const ldigraph *g, size_t from);


/**
 * Visits the given vertex in the given search of the given graph.
 *
 * @param g a pointer to a directed graph
 * @param s a search in that graph
 * @param curr a vertex in that graph
 */
static void ldigraph_dfs_visit(const ldigraph* g, ldigraph_search *s, size_t curr);


/**
 * Resizes the adjacency list for the given vertex in the given graph.
 * 
 * @param g a pointer to a directed graph
 * @param from the index of a vertex in that graph
 */
static void ldigraph_list_embiggen(ldigraph *g, size_t from);


/**
 * Prepares a search result for the given graph starting from the given
 * vertex.  It is the responsibility of the caller to destroy the result.
 *
 * @param g a pointer to a directed graph
 * @return a pointer to a search result
 */
static ldigraph_search *ldigraph_search_create(const ldigraph *g);


/**
 * Initializes a search result that has its associated graph set and
 * has space allocated for its arrays.
 *
 * @param s a pointer to a search result, non-NULL
 */
static void ldigraph_search_init(ldigraph_search *s);


/**
 * Destroys the given search result.
 *
 * @param s a pointer to a search result, non-NULL
 */
static void ldigraph_search_destroy(ldigraph_search *s);


ldigraph *ldigraph_create(size_t n)
{
  if (n < 1)
    {
      return NULL;
    }
  
  ldigraph *g = malloc(sizeof(ldigraph));
  if (g != NULL)
    {
      g->n = n;
      g->list_size = malloc(sizeof(size_t) * n);
      g->list_cap = malloc(sizeof(size_t) * n);
      g->adj = malloc(sizeof(size_t *) * n);
      
      if (g->list_size == NULL || g->list_cap == NULL || g->adj == NULL)
	{
	  free(g->list_size);
	  free(g->list_cap);
	  free(g->adj);
	  free(g);

	  return NULL;
	}

      for (size_t i = 0; i < n; i++)
	{
	  g->list_size[i] = 0;
	  g->adj[i] = malloc(sizeof(size_t) * LDIGRAPH_ADJ_LIST_INITIAL_CAPACITY);
	  g->list_cap[i] = g->adj[i] != NULL ? LDIGRAPH_ADJ_LIST_INITIAL_CAPACITY : 0;
	}
    }

  return g;
}


size_t ldigraph_size(const ldigraph *g)
{
  if (g != NULL)
    {
      return g->n;
    }
  else
    {
      return 0;
    }
}


void ldigraph_list_embiggen(ldigraph *g, size_t from)
{
  if (g->list_cap[from] != 0)
    {
      g->adj[from] = realloc(g->adj[from], sizeof(size_t *) * g->list_cap[from] * 2);
      g->list_cap[from] = g->adj[from] != NULL ? g->list_cap[from] * 2 : 0;
    }
}


void ldigraph_add_edge(ldigraph *g, size_t from, size_t to)
{
  if (g != NULL && from >= 0 && to >= 0 && from < g->n && to < g->n && from != to)
    {
      // make room if necessary
      if (g->list_size[from] == g->list_cap[from])
	{
	  ldigraph_list_embiggen(g, from);
	}

      // add to end of array if there is room
      if (g->list_size[from] < g->list_cap[from])
	{
	  g->adj[from][g->list_size[from]++] = to;
	}
    }
}


bool ldigraph_has_edge(const ldigraph *g, size_t from, size_t to)
{
  if (g != NULL && from < g->n && to < g->n && from != to)
    {
      // sequential search of from's adjacency list
      size_t i = 0;
      while (i < g->list_size[from] && g->adj[from][i] != to)
	{
	  i++;
	}
      return i < g->list_size[from];
    }
  else
    {
      return false;
    }
}


int ldigraph_shortest_path(const ldigraph *g, size_t from, size_t to)
{
  if (g == NULL || from >= g->n || to >= g->n)
    {
      return -1;
    }

  // do BFS starting from the from vertex
  ldigraph_search *s = ldigraph_bfs(g, from);

  if (s != NULL)
    {
      // look up the distance to the to vertex in the result and return it
      int shortest = s->dist[to];
      ldigraph_search_destroy(s);
      return shortest;
    }
  else
    {
      return -1;
    }
}


ldigraph_search *ldigraph_bfs(const ldigraph *g, size_t from)
{
  // REPLACE THIS STUB!
  if (g == NULL || from >= g->n){
    return NULL;
  }

  ldigraph_search *s = ldigraph_search_create(g);
  if(s == NULL){
    return NULL;
  }

  //allocate memory for queue(use array as queue)
  size_t *queue = malloc(sizeof(size_t) * g->n);
  if(queue == NULL){
    ldigraph_search_destroy(s);
    return NULL;
  }

  //init dist and color
  for(size_t i = 0; i < g->n; i++){
    s->dist[i] = -1;
    s->color[i] = LDIGRAPH_UNSEEN;
  }

  //BFS starting at index from
  size_t front_queue = 0;
  size_t back_queue = 0;
  //enque the indexes of vertices
  queue[back_queue++] = from;
  s->dist[from] = 0;
  s->color[from] =  LDIGRAPH_PROCESSING;

  while(front_queue < back_queue){
    //dequeue
    size_t current = queue[front_queue++];
    //get neighbors by checking adjacency list
    const size_t *adjacent_vertices = g->adj[current];

    for (size_t i = 0; i < g->list_size[current]; i++){
      size_t adjacent = adjacent_vertices[i];
      //check if neighbor is seen- if not mark it as processed
      if(s->color[adjacent] == LDIGRAPH_UNSEEN){
        s->color[adjacent] = LDIGRAPH_PROCESSING;

        s->dist[adjacent] = s->dist[current] + 1;
        s->pred[adjacent] = current;

        //now queue the adjacent vertex
        queue[back_queue++] = adjacent;
      }
    }
    s->color[current] = LDIGRAPH_DONE;
  }
  free(queue);
  return s;
}

int acyclic_helper(const ldigraph *g, size_t current, size_t to, int *length_track){
  //base case
  if(current == to){
    return 0;
  }

  if(length_track[current] != -2){
    return length_track[current];
  }

  //initialize longest path as -1
  int longest_dist = -1;

  const size_t *adjacent_vertices = g->adj[current];

  for (size_t i = 0; i < g->list_size[current]; i++){
    size_t adjacent = adjacent_vertices[i];
    int path_dist = acyclic_helper(g, adjacent, to, length_track);
    
    if (path_dist != -1){
      longest_dist = longest_dist > (path_dist + 1) ? longest_dist : (path_dist + 1);
    }
  }
  length_track[current] = longest_dist;
  return longest_dist;
}

int acyclic_longest_path(const ldigraph *g, size_t from, size_t to){
  int *length_track = malloc(sizeof(int) * g->n);
  if(length_track == NULL){
    return -1;
  }
  //initialize the array(to some invalid lengths first)
  for(size_t i = 0; i < g->n; i++){
    length_track[i] = -2;
  }

  int result = acyclic_helper(g, from, to, length_track);

  free(length_track);
  return result;
}

void cyclic_helper(const ldigraph *g, size_t current, size_t to, int length, int *longest_distance, int *color){
  if(current == to){
    if(length > *longest_distance){
      *longest_distance = length;
    }
    return;
  }
  //processing current vertex
  color[current] = LDIGRAPH_PROCESSING;

  const size_t *adjacent_vertices = g->adj[current];

  for (size_t i = 0; i < g->list_size[current]; i++){
    size_t adjacent = adjacent_vertices[i];
    if (color[adjacent] == LDIGRAPH_UNSEEN){
      cyclic_helper(g, adjacent, to, length + 1, longest_distance, color);
    }
  }
  //for backtracking
  color[current] = LDIGRAPH_UNSEEN;
}
int cyclic_longest_path(const ldigraph *g, size_t from, size_t to){
  //array to track seen vertices
  int *color = malloc(sizeof(int) * g->n);
  if(color == NULL){
    return -1;
  }

  for(size_t i = 0; i < g->n; i++){
    color[i] = LDIGRAPH_UNSEEN;
  }

  int longest_dist = -1;

  cyclic_helper(g, from, to, 0, &longest_dist, color);

  free(color);
  return longest_dist;
}


int ldigraph_longest_path(const ldigraph *g, size_t from, size_t to)
{
  if (g == NULL || from >= g->n || to >= g->n)
    {
      return -1;
    }
  if (from == to){
    return 0;
  }

  // do a DFS to determine if there is a cycle (need to modify DFS to
  // find cycles)
  check_cycle = false;

  ldigraph_search *s = ldigraph_dfs(g, from);
  if (s == NULL){
    return -1;
  }

  bool is_cyclic = check_cycle;

  // If acyclic: do a version of DFS that keeps track, for each vertex v,
  // of the longest simple path from v to the destination; update the
  // length of the longest path every time you're finished looking at an
  // edge, regardless of whether you traversed it or not
  
  // If cyclic: brute force search on cyclic graph starting from from vertex,
  // going down all simple paths
  
  ldigraph_search_destroy(s);

  if(is_cyclic){
    return cyclic_longest_path(g, from, to);
  }else{
    return acyclic_longest_path(g,from, to);
  }
}


ldigraph_search *ldigraph_dfs(const ldigraph *g, size_t from)
{
  if (g == NULL || from >= g->n)
    {
      return NULL;
    }

  ldigraph_search *s = ldigraph_search_create(g);
  if (s != NULL)
    {
      // start at from
      // (note we do not have the restart-if-some-vertices-unvisited
      // loop here; consider whether you will need it)
      s->dist[from] = 0;
      ldigraph_dfs_visit(g, s, from);
    }
  return s;
}


ldigraph_search *ldigraph_dfs_with_restart(const ldigraph *g)
{
  if (g == NULL)
    {
      return NULL;
    }

  ldigraph_search *s = ldigraph_search_create(g);
  if (s != NULL)
    {
      // try all starting points for DFS
      for (size_t from = 0; from < g->n; from++)
	{
	  // use from as a starting point if no previous search found it
	  if (s->color[from] == LDIGRAPH_UNSEEN)
	    {
	      s->dist[from] = 0;
	      ldigraph_dfs_visit(g, s, from);
	    }
	}
    }
  return s;
}



void ldigraph_dfs_visit(const ldigraph* g, ldigraph_search *s, size_t curr)
{
  s->color[curr] = LDIGRAPH_PROCESSING;

  // make alias for adjacency list for current vertex
  const size_t *neighbors = g->adj[curr];
  
  // iterate over outgoing edges
  for (size_t i = 0; i < g->list_size[curr]; i++)
    {
      size_t to = neighbors[i];
      if (s->color[to] == LDIGRAPH_UNSEEN)
	{
	  // found an edge to a new vertex -- explore it
	  s->dist[to] = s->dist[curr] + 1;
	  s->pred[to] = curr;
	  
	  ldigraph_dfs_visit(g, s, to);
	} else if(s->color[to] == LDIGRAPH_PROCESSING){
    check_cycle = true;
    return;
  }
    }
  
  // mark and record current vertex finished
  s->color[curr] = LDIGRAPH_DONE;
}


void ldigraph_destroy(ldigraph *g)
{
  if (g != NULL)
    {
      for (size_t i = 0; i < g->n; i++)
	{
	  free(g->adj[i]);
	}
      free(g->adj);
      free(g->list_cap);
      free(g->list_size);
      free(g);
    }
}


ldigraph_search *ldigraph_search_create(const ldigraph *g)
{
  if (g != NULL)
    {
      ldigraph_search *s = malloc(sizeof(ldigraph_search));
      
      if (s != NULL)
	{
	  s->g = g;
	  s->color = malloc(sizeof(int) * g->n);
	  s->dist = malloc(sizeof(int) * g->n);
	  s->pred = malloc(sizeof(int) * g->n);

	  if (s->color != NULL && s->dist != NULL && s->pred != NULL)
	    {
	      ldigraph_search_init(s);
	    }
	  else
	    {
	      free(s->pred);
	      free(s->dist);
	      free(s->color);
	      free(s);
	      return NULL;
	    }
	}

      return s;
    }
  else
    {
      return NULL;
    }
}


void ldigraph_search_init(ldigraph_search *s)
{
  // initialize all vertices to unseen
  for (size_t i = 0; i < s->g->n; i++)
    {
      s->color[i] = LDIGRAPH_UNSEEN;
      s->dist[i] = -1; // -1 for no path yet
      s->pred[i] = -1; // no predecessor yet
    }
}


void ldigraph_search_destroy(ldigraph_search *s)
{
  if (s != NULL)
    {
      free(s->color);
      free(s->dist);
      free(s->pred);
      free(s);
    }
}
