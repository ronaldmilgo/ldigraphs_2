#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ldigraph.h"

/**
 * Reads and returns the graph contained in the given file.
 * Returns NULL if the file could not be read or if the
 * graph could not be created.
 *
 * @param fname the name of the file containing the graph
 * @return a pointer to the graph build
 */
ldigraph *read_graph(const char *fname);


/**
 * Creates a sparse acyclic graph with the given number of vertices.
 * The graph will have vertices numbered 0,...,size-1 with vertices
 * 2, ..., size-1 having an edge to as many of the next three vertices
 * as exist.  As a result, there will be a path 0, 2, ..., size-1, 1
 * in the graph.
 *
 * @param size an integer at least 2
 * @return a pointer to a graph, or NULL for a memory allocation error
 */
ldigraph *create_sparse(size_t size);


/**
 * Returns a pointer to the graph path finding function specified by the
 * given string.  The string may be "-shortest" or "-longest"
 * to specify finding the shortest or longest path respectively.
 *
 * @param s a string, non-NULL
 * @return a pointer to the corresponding function
 */
int (*determine_method(const char *s))(const ldigraph*, size_t, size_t);


int main(int argc, char **argv)
{
  if (argc < 2)
    {
      fprintf(stderr, "USAGE: %s filename [[method from to...]...]\n", argv[0]);
      return 1;
    }

  ldigraph *g;
  if (strcmp(argv[1], "-timing") == 0)
    {
      int size;
      if (argc < 4 || (size = atoi(argv[argc - 2])) <= 0)
	{
	  fprintf(stderr, "USAGE: %s -timing [[method from to...]...] size on/off\n", argv[0]);
	  return 1;
	}
      int on = atoi(argv[argc - 1]);

      g = create_sparse(size);
      if (g == NULL)
	{
	  return 1;
	}
      
      if (on != 1)
	{
	  // skip the rest for baseline timing test
	  ldigraph_destroy(g);
	  return 0;
	}
      
      // ignore last two arguments (size and on/off)
      argc -= 2;
    }
  else
    {
      // read graph from file
      g = read_graph(argv[1]);
    }

  if (g != NULL)
    {
      size_t a = 2;
      while (a + 2 < argc)
	{
	  // determine search method
	  int (*find_path)(const ldigraph *, size_t, size_t) = determine_method(argv[a]);

	  if (find_path != NULL)
	    {
	      // get from vertex
	      int from = atoi(argv[a + 1]);
	      int to = atoi(argv[a + 2]);

	      // check whether vertices are legal
	      if (from >= 0 && from < ldigraph_size(g) && to >= 0 && to < ldigraph_size(g))
		{
		  // do the search 
		  int length = find_path(g, from, to);

		  // print answer
		  printf("%9.9s: %3d ~> %3d: %d\n", argv[a], from, to, length);
		}
	    }
	  
	  // go on to next method
	  a += 3;
	}
      
      ldigraph_destroy(g);
    }

  return 0;
}


ldigraph *read_graph(const char *fname)
{
  FILE *in = fopen(fname, "r");
  ldigraph *g = NULL;

  if (in != NULL)
    {
      size_t size;
      if (fscanf(in, "%zu", &size) == 1)
	{
	  g = ldigraph_create(size);

	  if (g != NULL)
	    {
	      int from, to;
	      while (fscanf(in, "%d %d", &from, &to) == 2)
		{
		  if (from >= 0 && from < size && to >= 0 && to < size)
		    {
		      ldigraph_add_edge(g, from, to);
		    }
		}
	    }
	}
      
      fclose(in);
    }

  return g;
}

int (*determine_method(const char *s))(const ldigraph*, size_t, size_t)
{
  if (strcmp(s, "-shortest") == 0)
    {
      return ldigraph_shortest_path;
    }
  else if (strcmp(s, "-longest") == 0)
    {
      return ldigraph_longest_path;
    }
  else
    {
      return NULL;
    }
}

ldigraph *create_sparse(size_t size)
{
  // make a sparse graph for timing -shortest and -longest on acyclic
  ldigraph *g = ldigraph_create(size);
  
  if (g == NULL)
    {
      return NULL;
    }
  
  // main spine of the graph will be 0->2->3->...->size-1->1
  // with additional edges from the vertices between 2 and size-2
  
  // vertex 0 will be the source vertex (1st in topo sort)
  ldigraph_add_edge(g, 0, 2);
  
  // vertex 1 will be the destination vertex (last in topo sort)
  ldigraph_add_edge(g, size - 1, 1);
  
  // loop over other vertices (2,...,size-2)
  for (size_t u = 2; u < size - 1; u++)
    {
      // each of these vertices has edges to the next three
      // (as long as that doesn't go past vertex size-1)
      size_t dest[3];
      size_t dest_count = 0;
      dest[dest_count++] = u + 1;
      for (size_t k = 2; k <= 3; k++)
	{
	  if (u < size - k)
	    {
	      dest[dest_count++] = u + k;
	    }
	}
      
      // shuffle order of edges to make sure students' code doesn't
      // depend on a particular ordering of the adjacency lists
      size_t swap = u % 3;
      if (swap > 0 && swap < dest_count)
	{
	  size_t temp = dest[0];
	  dest[0] = dest[swap];
	  dest[swap] = temp;
	}
      
      // add edges to the graph
      for (size_t i = 0; i < dest_count; i++)
	{
	  ldigraph_add_edge(g, u, dest[i]);
	}
    }
  
  return g;
}
