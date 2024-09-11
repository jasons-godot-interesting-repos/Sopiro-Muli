//LICENSE:  This code (AabbTree.cs) is made available under the MPL-2.0
//LICENSE:  https://www.tldrlegal.com/license/mozilla-public-license-2-0-mpl-2
//LICENSE:  Summary: If you make changes to THIS CODE, the changes must be made public.
//LICENSE:  You can use this in ANY project, commercial or otherwise.


using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using Godot;

/// <summary>
/// Provides extension methods for the Aabb struct to support AABB tree operations.
/// </summary>
/// <remarks>
/// These extensions enhance the functionality of Godot's built-in Aabb struct,
/// adding methods necessary for efficient AABB tree operations and collision detection.
/// </remarks>
public static class AabbExtensions
{
	/// <summary>
	/// Calculates the surface area of the AABB.
	/// </summary>
	/// <param name="aabb">The AABB to calculate the area for.</param>
	/// <returns>The surface area of the AABB.</returns>
	/// <remarks>
	/// This method is crucial for the Surface Area Heuristic (SAH) used in optimizing the AABB tree.
	/// A smaller surface area generally leads to more efficient collision detection.
	/// </remarks>
	public static float Area(this Aabb aabb)
	{
		float xSize = aabb.Size.X;
		float ySize = aabb.Size.Y;
		float zSize = aabb.Size.Z;

		return 2.0f * ((xSize * ySize) + (xSize * zSize) + (ySize * zSize));
	}

	/// <summary>
	/// Calculates the perimeter of the AABB.
	/// </summary>
	/// <param name="aabb">The AABB to calculate the perimeter for.</param>
	/// <returns>The perimeter of the AABB.</returns>
	/// <remarks>
	/// The perimeter is used as an alternative to surface area in some AABB tree optimizations.
	/// It can be faster to compute than the full surface area while still providing a good heuristic.
	/// </remarks>
	public static float _Perimeter(this Aabb aabb)
	{
		var size = aabb.Size;
		return 4.0f * (size.X + size.Y + size.Z);
	}

	/// <summary>
	/// Performs a ray cast against the AABB.
	/// </summary>
	/// <param name="aabb">The AABB to test against.</param>
	/// <param name="from">The start point of the ray.</param>
	/// <param name="to">The end point of the ray.</param>
	/// <param name="tMin">The minimum distance along the ray to test (default is 0).</param>
	/// <param name="tMax">The maximum distance along the ray to test (default is 1).</param>
	/// <param name="halfExtents">Half-extents to expand the AABB by (default is zero).</param>
	/// <returns>The distance to the intersection point, or float.MaxValue if no intersection.</returns>
	/// <remarks>
	/// This method is essential for ray casting operations in the AABB tree, allowing for
	/// efficient pruning of the search space during tree traversal.
	/// </remarks>
	public static float _RayCast(this Aabb aabb, Vector3 from, Vector3 to, float tMin = 0f, float tMax = 1f, Vector3 halfExtents = default)
	{
		Vector3 min = aabb.Position - halfExtents;
		Vector3 max = aabb.End + halfExtents;

		Vector3 direction = to - from;
		// Calculate inverse direction to avoid division in the loop
		Vector3 directionInv = new Vector3(
			  direction.X != 0 ? 1.0f / direction.X : float.MaxValue,
			  direction.Y != 0 ? 1.0f / direction.Y : float.MaxValue,
			  direction.Z != 0 ? 1.0f / direction.Z : float.MaxValue
		);

		// Calculate intersection with each pair of planes
		float tX1 = (min.X - from.X) * directionInv.X;
		float tX2 = (max.X - from.X) * directionInv.X;
		float tY1 = (min.Y - from.Y) * directionInv.Y;
		float tY2 = (max.Y - from.Y) * directionInv.Y;
		float tZ1 = (min.Z - from.Z) * directionInv.Z;
		float tZ2 = (max.Z - from.Z) * directionInv.Z;

		// Find the largest minimum and smallest maximum t values
		float tNear = Mathf.Max(Mathf.Max(Mathf.Min(tX1, tX2), Mathf.Min(tY1, tY2)), Mathf.Min(tZ1, tZ2));
		float tFar = Mathf.Min(Mathf.Min(Mathf.Max(tX1, tX2), Mathf.Max(tY1, tY2)), Mathf.Max(tZ1, tZ2));

		// Check if the ray misses the box
		if (tNear > tFar || tFar < 0)
		{
			return float.MaxValue;
		}

		// Clamp to the segment's endpoints
		tNear = Mathf.Max(tNear, tMin);
		tFar = Mathf.Min(tFar, tMax);

		// Return the nearest non-negative intersection distance
		if (tNear <= tFar && tFar >= 0)
		{
			return tNear;
		}

		return float.MaxValue;
	}
}

/// <summary>
/// Defines the interface for objects that can be stored in the AABB tree.
/// </summary>
/// <remarks>
/// This interface ensures that any object stored in the tree can provide its own AABB,
/// allowing for flexible use of the tree with various object types.
/// </remarks>
public interface ICollider
{
	/// <summary>
	/// Gets the Axis-Aligned Bounding Box (AABB) for this collider.
	/// </summary>
	/// <returns>The AABB enclosing this collider.</returns>
	Aabb GetAabb();
}

/// <summary>
/// Represents a dynamic Axis-Aligned Bounding Box (AABB) tree for efficient spatial queries.
/// </summary>
/// <typeparam name="TData">The type of data stored in the tree nodes.</typeparam>
/// <remarks>
/// This class implements a self-balancing AABB tree that supports dynamic updates,
/// efficient spatial queries, and maintains an optimal structure using the Surface Area Heuristic (SAH).
/// It's particularly useful for broad-phase collision detection in physics simulations and
/// spatial partitioning in game engines.
/// </remarks>
public partial class AABBTree<TData> where TData : class
{
	/// <summary>
	/// Represents a constant value indicating a null node in the tree.
	/// </summary>
	/// <remarks>
	/// This value is used to represent the absence of a node, similar to a null pointer in C++.
	/// It's used extensively throughout the tree operations to indicate leaf nodes or the end of a branch.
	/// </remarks>
	private const int NULL_NODE = -1;

	/// <summary>
	/// The margin added to AABBs to provide some padding and reduce frequent updates.
	/// </summary>
	/// <remarks>
	/// This margin helps to reduce the frequency of updates needed when objects move slightly,
	/// improving performance for dynamic scenes with many moving objects.
	/// </remarks>
	private float AabbMargin = 0.1f;

	/// <summary>
	/// A multiplier used to expand AABBs in the direction of movement.
	/// </summary>
	/// <remarks>
	/// This multiplier is used to predict future positions of moving objects,
	/// further reducing the need for frequent updates and improving overall performance.
	/// </remarks>
	private float AabbMultiplier = 5.0f;

	/// <summary>
	/// A concurrent dictionary mapping data objects to their corresponding node indices in the tree.
	/// </summary>
	/// <remarks>
	/// This lookup table allows for fast access to nodes given their associated data,
	/// which is crucial for operations like updates and removals. The use of a ConcurrentDictionary
	/// allows for thread-safe access in multi-threaded scenarios.
	/// </remarks>
	internal Dictionary<TData, int> nodeIndexLookup = new Dictionary<TData, int>();

	/// <summary>
	/// Represents a node in the AABB tree.
	/// </summary>
	/// <remarks>
	/// This struct encapsulates all the data needed for each node in the tree,
	/// including its AABB, parent-child relationships, and associated object data.
	/// The use of a struct here optimizes memory layout and access patterns.
	/// </remarks>
	public record struct Node
	{
		/// <summary>
		/// The Axis-Aligned Bounding Box for this node.
		/// </summary>
		public Aabb Aabb;

		/// <summary>
		/// The index of this node's parent in the nodes array.
		/// </summary>
		public int Parent;

		/// <summary>
		/// The index of this node's first child in the nodes array.
		/// </summary>
		public int Child1;

		/// <summary>
		/// The index of this node's second child in the nodes array.
		/// </summary>
		public int Child2;

		/// <summary>
		/// The index of the next node in the free list.
		/// </summary>
		public int Next;

		/// <summary>
		/// Indicates whether this node has been moved since the last update.
		/// </summary>
		public bool Moved;

		/// <summary>
		/// The data object associated with this node (null for internal nodes).
		/// </summary>
		public TData? Data;

		/// <summary>
		/// The original AABB of the item, without any enlargement.
		/// </summary>
		public Aabb ItemAabb;

		/// <summary>
		/// Indicates whether this node is a leaf node.
		/// </summary>
		public bool IsLeaf => Child1 == NULL_NODE;
	}

	/// <summary>
	/// The index of the root node in the nodes array.
	/// </summary>
	/// <remarks>
	/// The root node is the top-level node of the tree. A value of NULL_NODE
	/// indicates an empty tree.
	/// </remarks>
	private int root;

	/// <summary>
	/// Gets the AABB of the entire tree.
	/// </summary>
	/// <remarks>
	/// This property returns the AABB of the root node, which encompasses
	/// all objects in the tree. It's useful for broad checks or visualization.
	/// </remarks>
	public Aabb TreeAabb => root == NULL_NODE ? default : nodes[root].Aabb;

	/// <summary>
	/// The array storing all nodes in the tree.
	/// </summary>
	/// <remarks>
	/// This array is the core data structure of the tree. It's managed internally
	/// to grow as needed and uses a free list for efficient memory management.
	/// </remarks>
	internal Node[] nodes;

	/// <summary>
	/// The current capacity of the nodes array.
	/// </summary>
	/// <remarks>
	/// This value represents the total size of the nodes array, including both
	/// active and free nodes. It's used to manage array growth.
	/// </remarks>
	private int nodeCapacity;

	/// <summary>
	/// The current number of active nodes in the tree.
	/// </summary>
	/// <remarks>
	/// This count represents the number of nodes actually in use in the tree,
	/// which is less than or equal to nodeCapacity.
	/// </remarks>
	private int nodeCount;

	/// <summary>
	/// Gets the number of objects stored in the tree.
	/// </summary>
	/// <remarks>
	/// This property returns the count of leaf nodes in the tree, which corresponds
	/// to the number of objects stored. It performs a consistency check to ensure
	/// the tree is in a valid state.
	/// </remarks>
	public int Count
	{
		get
		{
			this._AssertTreeOk();
			//__.Assert(nodeCount == nodeIndexLookup.Count);
			return nodeIndexLookup.Count;
		}
	}

	/// <summary>
	/// The index of the first node in the free list.
	/// </summary>
	/// <remarks>
	/// The free list is a linked list of unused node indices, used for efficient
	/// allocation and deallocation of nodes without frequent resizing of the nodes array.
	/// </remarks>
	private int freeList;

	/// <summary>
	/// Calculates the surface area of an AABB.
	/// </summary>
	/// <param name="aabb">The AABB to calculate the surface area for.</param>
	/// <returns>The surface area of the AABB.</returns>
	/// <remarks>
	/// This method is crucial for the Surface Area Heuristic (SAH) used in tree optimizations.
	/// It uses the perimeter as an approximation of surface area for faster computation.
	/// </remarks>
	private float SurfaceArea(Aabb aabb)
	{
		return aabb._Perimeter();
	}

	/// <summary>
	/// Initializes a new instance of the AABBTree class.
	/// </summary>
	/// <remarks>
	/// This constructor sets up the initial state of the tree with a small capacity.
	/// The tree will grow as needed when more nodes are added.
	/// </remarks>
	public AABBTree()
	{
		root = NULL_NODE;
		nodeCapacity = 5;
		nodeCount = 0;

		nodes = new Node[nodeCapacity];
		// Initialize the free list
		for (int i = 0; i < nodeCapacity; i++)
		{
			nodes[i] = new Node();
			nodes[i].Next = i + 1;
			nodes[i].Parent = i;
		}
		nodes[nodeCapacity - 1].Next = NULL_NODE;
		nodes[nodeCapacity - 1].Parent = nodeCapacity - 1;

		freeList = 0;
	}

	/// <summary>
	/// Creates a new node in the tree for the given data and AABB.
	/// </summary>
	/// <param name="data">The data object to store in the new node.</param>
	/// <param name="aabb">The AABB of the new node.</param>
	/// <remarks>
	/// This method is the primary way to add new objects to the tree. It creates a new
	/// leaf node, inserts it into the tree structure, and updates the lookup dictionary.
	/// </remarks>
	public void CreateNode(TData data, Aabb aabb)
	{
		int index = CreateNodeInternal(data, aabb);
		nodeIndexLookup.TryAdd(data, index);
	}

	/// <summary>
	/// Internal method to create a new node in the tree.
	/// </summary>
	/// <param name="data">The data object to store in the new node.</param>
	/// <param name="aabb">The AABB of the new node.</param>
	/// <returns>The index of the newly created node.</returns>
	/// <remarks>
	/// This method handles the actual creation of the node, including allocating
	/// a new node, setting its properties, and inserting it into the tree structure.
	/// </remarks>
	private int CreateNodeInternal(TData data, Aabb aabb)
	{
		int newNode = AllocateNode();

		// Enlarge the AABB by the margin to reduce frequent updates
		Vector3 r = new Vector3(AabbMargin, AabbMargin, AabbMargin);
		nodes[newNode].Aabb = new Aabb(aabb.Position - r, aabb.Size + r + r);
		nodes[newNode].ItemAabb = aabb;
		nodes[newNode].Data = data;
		nodes[newNode].Parent = NULL_NODE;
		nodes[newNode].Moved = true;

		InsertLeaf(newNode);

		return newNode;
	}

	/// <summary>
	/// Moves a node in the tree, updating its AABB.
	/// </summary>
	/// <param name="data">The data object associated with the node to move.</param>
	/// <param name="aabb">The new AABB for the node.</param>
	/// <param name="autoDisplacement">If true, calculates displacement automatically.</param>
	/// <param name="forceMove">If true, forces the move even if the new AABB is contained in the old one.</param>
	/// <returns>True if the node was moved, false otherwise.</returns>
	/// <remarks>
	/// This method is used to update the position of an object in the tree. It handles
	/// both small movements (which might not require tree restructuring) and larger
	/// movements that may require reinsertion of the node.
	/// </remarks>
	public bool MoveNode(TData data, Aabb aabb, bool autoDisplacement = false, bool forceMove = false)
	{
		if (!nodeIndexLookup.TryGetValue(data, out int node))
		{
			throw __.Throw("item not found in AabbTree");
		}

		Vector3 displacement;
		if (autoDisplacement is false)
		{
			displacement = Vector3.Zero;
		}
		else
		{
			var oldAabb = nodes[node].ItemAabb;
			displacement = aabb.GetCenter() - oldAabb.GetCenter();
		}

		return MoveNodeInternal(node, aabb, displacement, forceMove);
	}

	/// <summary>
	/// Moves a node in the tree, updating its AABB with a specified displacement.
	/// </summary>
	/// <param name="data">The data object associated with the node to move.</param>
	/// <param name="aabb">The new AABB for the node.</param>
	/// <param name="displacement">The displacement vector of the node.</param>
	/// <param name="forceMove">If true, forces the move even if the new AABB is contained in the old one.</param>
	/// <returns>True if the node was moved, false otherwise.</returns>
	/// <remarks>
	/// This overload allows for specifying a custom displacement vector, which can be
	/// useful for predicting future positions or handling specific movement patterns.
	/// </remarks>
	public bool MoveNode(TData data, Aabb aabb, Vector3 displacement, bool forceMove = false)
	{
		if (!nodeIndexLookup.TryGetValue(data, out int node))
		{
			throw __.Throw("item not found in AabbTree");
		}

		return MoveNodeInternal(node, aabb, displacement, forceMove);
	}

	/// <summary>
	/// Internal method to move a node in the tree.
	/// </summary>
	/// <param name="node">The index of the node to move.</param>
	/// <param name="itemAabb">The new AABB for the item.</param>
	/// <param name="displacement">The displacement vector of the node.</param>
	/// <param name="forceMove">If true, forces the move even if the new AABB is contained in the old one.</param>
	/// <returns>True if the node was moved, false otherwise.</returns>
	/// <remarks>
	/// This method handles the actual logic of moving a node, including updating its AABB,
	/// checking if reinsertion is necessary, and maintaining tree consistency.
	/// </remarks>
	private bool MoveNodeInternal(int node, Aabb itemAabb, Vector3 displacement, bool forceMove = false)
	{
		_AssertTreeOk();

		//if (node < 0 || node >= nodeCapacity) return false;
		//if (!nodes[node].IsLeaf) return false;
		__.Throw(node >= 0 && node < nodeCapacity);
		__.Throw(nodes[node].IsLeaf);

		//update our item location
		nodes[node].ItemAabb = itemAabb;

		Aabb treeAabb = nodes[node].Aabb;
		if (treeAabb.Encloses(itemAabb) && !forceMove) return false;

		RemoveLeaf(node);

		// Enlarge the AABB by the margin
		Vector3 r = new Vector3(AabbMargin, AabbMargin, AabbMargin);
		var aabb = new Aabb(itemAabb.Position - r, itemAabb.Size + r + r);

		// Expand AABB in the direction of movement
		Vector3 d = displacement * AabbMultiplier;
		var finalAabb = aabb._Union(itemAabb with
		{
			Position = itemAabb.Position + d,
		});
		aabb = finalAabb;

		//if (d.X < 0.0f) aabb.Position -= new Vector3(d.X, 0, 0);
		//else aabb.Size += new Vector3(d.X, 0, 0);

		//if (d.Y < 0.0f) aabb.Position -= new Vector3(0, d.Y, 0);
		//else aabb.Size += new Vector3(0, d.Y, 0);

		//if (d.Z < 0.0f) aabb.Position -= new Vector3(0, 0, d.Z);
		//else aabb.Size += new Vector3(0, 0, d.Z);

		nodes[node].Aabb = aabb;

		InsertLeaf(node);
		nodes[node].Moved = true;

		_AssertTreeOk();
		return true;
	}

	/// <summary>
	/// Removes a node from the tree.
	/// </summary>
	/// <param name="data">The data object associated with the node to remove.</param>
	/// <remarks>
	/// This method removes a node from the tree and updates the tree structure accordingly.
	/// It also removes the data from the lookup dictionary.
	/// </remarks>
	public void RemoveNode(TData data)
	{
		if (nodeIndexLookup._TryRemove(data, out int node))
		{
			RemoveNodeInternal(node);
		}
		else
		{
			throw __.Throw("item not found in AabbTree");
		}
	}

	/// <summary>
	/// Internal method to remove a node from the tree.
	/// </summary>
	/// <param name="node">The index of the node to remove.</param>
	/// <remarks>
	/// This method handles the actual removal of the node from the tree structure,
	/// including updating parent-child relationships and freeing the node.
	/// </remarks>
	private void RemoveNodeInternal(int node)
	{
		//if (node < 0 || node >= nodeCapacity) return;
		//if (!nodes[node].IsLeaf) return;
		__.Throw(node >= 0 && node < nodeCapacity);
		__.Throw(nodes[node].IsLeaf);

		RemoveLeaf(node);
		FreeNode(node);
	}

	/// <summary>
	/// Performs a point query on the tree, invoking a callback for each intersecting object.
	/// </summary>
	/// <param name="point">The query point.</param>
	/// <param name="callback">A callback function to invoke for each intersecting object.</param>
	/// <remarks>
	/// This method traverses the tree to find all objects whose AABBs contain the given point.
	/// The callback can be used to perform additional checks or process the found objects.
	/// </remarks>
	public void Query(Vector3 point, Func<TData, bool> callback)
	{
		QueryInternal(point, (_, data) => callback(data));
	}

	/// <summary>
	/// Internal method to perform a point query on the tree.
	/// </summary>
	/// <param name="point">The query point.</param>
	/// <param name="callback">A callback function to invoke for each intersecting node.</param>
	/// <remarks>
	/// This method implements the actual tree traversal for point queries, using a stack
	/// to avoid recursion and improve performance.
	/// </remarks>
	private void QueryInternal(Vector3 point, Func<int, TData, bool> callback)
	{
		if (root == NULL_NODE) return;

		var stack = new Stack<int>();
		stack.Push(root);

		while (stack.Count > 0)
		{
			int current = stack.Pop();

			if (!nodes[current].Aabb.HasPoint(point)) continue;

			if (nodes[current].IsLeaf)
			{
				if (!callback(current, nodes[current].Data))
				{
					return;
				}
			}
			else
			{
				stack.Push(nodes[current].Child1);
				stack.Push(nodes[current].Child2);
			}
		}
	}

	/// <summary>
	/// Performs an AABB query on the tree, invoking a callback for each intersecting object.
	/// </summary>
	/// <param name="aabb">The query AABB.</param>
	/// <param name="callback">A callback function to invoke for each intersecting object.</param>
	/// <remarks>
	/// This method traverses the tree to find all objects whose AABBs overlap with the given AABB.
	/// The callback can be used to perform additional checks or process the found objects.
	/// </remarks>
	public void Query(Aabb aabb, Func<TData, bool> callback)
	{
		QueryInternal(aabb, (_, data) => callback(data));
	}

	/// <summary>
	/// Internal method to perform an AABB query on the tree.
	/// </summary>
	/// <param name="aabb">The query AABB.</param>
	/// <param name="callback">A callback function to invoke for each intersecting node.</param>
	/// <remarks>
	/// This method implements the actual tree traversal for AABB queries, using a stack
	/// to avoid recursion and improve performance.
	/// </remarks>
	private void QueryInternal(Aabb aabb, Func<int, TData, bool> callback)
	{
		if (root == NULL_NODE) return;

		var stack = new Stack<int>();
		stack.Push(root);

		while (stack.Count > 0)
		{
			int current = stack.Pop();

			if (!nodes[current].Aabb.Intersects(aabb)) continue;

			if (nodes[current].IsLeaf)
			{
				if (!callback(current, nodes[current].Data))
				{
					return;
				}
			}
			else
			{
				stack.Push(nodes[current].Child1);
				stack.Push(nodes[current].Child2);
			}
		}
	}

	/// <summary>
	/// Inserts a leaf node into the tree.
	/// </summary>
	/// <param name="leaf">The index of the leaf node to insert.</param>
	/// <returns>The index of the inserted leaf node.</returns>
	/// <remarks>
	/// This method is a crucial part of tree construction and modification. It finds the best
	/// sibling for the new leaf using a surface area heuristic, creates a new parent node,
	/// and adjusts the tree structure accordingly. After insertion, it performs a bottom-up
	/// update of the tree to maintain optimal structure.
	/// </remarks>
	private int InsertLeaf(int leaf)
	{
		Debug.Assert(0 <= leaf && leaf < nodeCapacity);
		Debug.Assert(nodes[leaf].IsLeaf);

		if (root == NULL_NODE)
		{
			root = leaf;
			return leaf;
		}

		Aabb aabb = nodes[leaf].Aabb;

		// Find the best sibling for the new leaf
		int bestSibling = root;
		float bestCost = SurfaceArea(nodes[root].Aabb.Merge(aabb));

		var stack = new Stack<(int node, float inheritedCost)>();
		stack.Push((root, 0.0f));

		while (stack.Count != 0)
		{
			var (currentNode, inheritedCost) = stack.Pop();

			Aabb combined = nodes[currentNode].Aabb.Merge(aabb);
			float directCost = SurfaceArea(combined);
			float cost = directCost + inheritedCost;

			if (cost < bestCost)
			{
				bestCost = cost;
				bestSibling = currentNode;
			}

			// Compute the cost of descending further down the tree
			inheritedCost += directCost - SurfaceArea(nodes[currentNode].Aabb);
			float lowerBoundCost = SurfaceArea(aabb) + inheritedCost;

			if (lowerBoundCost < bestCost)
			{
				if (!nodes[currentNode].IsLeaf)
				{
					stack.Push((nodes[currentNode].Child1, inheritedCost));
					stack.Push((nodes[currentNode].Child2, inheritedCost));
				}
			}
		}

		// Create a new parent
		int oldParent = nodes[bestSibling].Parent;
		int newParent = AllocateNode();
		nodes[newParent].Aabb = nodes[bestSibling].Aabb.Merge(aabb);
		nodes[newParent].Data = null;
		nodes[newParent].Parent = oldParent;

		// Connect new leaf and sibling to new parent
		nodes[newParent].Child1 = leaf;
		nodes[newParent].Child2 = bestSibling;
		nodes[leaf].Parent = newParent;
		nodes[bestSibling].Parent = newParent;

		if (oldParent != NULL_NODE)
		{
			// The sibling was not the root
			if (nodes[oldParent].Child1 == bestSibling)
			{
				nodes[oldParent].Child1 = newParent;
			}
			else
			{
				nodes[oldParent].Child2 = newParent;
			}
		}
		else
		{
			// The sibling was the root
			root = newParent;
		}
		_AssertTreeOk(ignoreAabbConsistency: true);

		// Walk back up the tree refitting AABBs and applying rotations
		int ancestor = newParent;
		while (ancestor != NULL_NODE)
		{
			int child1 = nodes[ancestor].Child1;
			int child2 = nodes[ancestor].Child2;

			nodes[ancestor].Aabb = nodes[child1].Aabb.Merge(nodes[child2].Aabb);

			Rotate(ancestor);

			ancestor = nodes[ancestor].Parent;
		}
		_AssertTreeOk();
		return leaf;
	}

	/// <summary>
	/// Removes a leaf node from the AABB tree.
	/// </summary>
	/// <param name="leaf">The index of the leaf node to remove.</param>
	/// <remarks>
	/// This method handles the removal of a leaf node from the tree, including updating
	/// the tree structure and rebalancing as necessary. After this call, the leaf node
	/// is in an orphaned state and should be either freed or reinserted.
	/// </remarks>
	private void RemoveLeaf(int leaf)
	{
		// 1. Assertions and Preconditions:
		__.Assert(leaf >= 0 && leaf < nodeCapacity); // Ensure the leaf index is valid.
		__.Assert(nodes[leaf].IsLeaf); // Ensure the node being removed is actually a leaf.

		_AssertTreeOk(ignoreAabbConsistency: true); // Assert that the tree is in a valid state before the operation.

		// 2. Handle Root Removal:
		int parent = nodes[leaf].Parent;
		if (parent == NULL_NODE) // If the leaf is the root node...
		{
			__.Assert(root == leaf); // Assert that the root is indeed the leaf.
			root = NULL_NODE; // Set the root to NullNode, indicating an empty tree.
			return; // The removal is complete.
		}

		// 3. Identify Grandparent and Sibling:
		int grandParent = nodes[parent].Parent;
		//int sibling = (nodes[parent].Child1 == leaf) ? nodes[parent].Child2 : nodes[parent].Child1;
		//// **Potential issue:** This assumes the leaf is either the first or second child. 
		//// It should be corrected to handle cases where the leaf is neither.
		int sibling = NULL_NODE;
		if (nodes[parent].Child1 == leaf)
		{
			sibling = nodes[parent].Child2;
		}
		else if (nodes[parent].Child2 == leaf)
		{
			sibling = nodes[parent].Child1;
		}
		else
		{
			// This should never happen in a valid tree, but it's good to have a check
			throw __.Throw("Removed leaf is not a child of its parent!");
		}


		// 4. Free the Parent Node:
		FreeNode(parent); // The parent node is no longer needed since the leaf is removed.

		// 5. Update Tree Structure:
		if (grandParent != NULL_NODE) // If the removed leaf has a grandparent...
		{
			// a. Connect Sibling to Grandparent:
			nodes[sibling].Parent = grandParent; // The sibling becomes a child of the grandparent.
			if (nodes[grandParent].Child1 == parent)
				nodes[grandParent].Child1 = sibling; // Replace the parent with the sibling.
			else
				nodes[grandParent].Child2 = sibling;

			// b. Update Ancestors and Balance:
			int ancestor = grandParent;
			while (ancestor != NULL_NODE) // Traverse up the tree from the grandparent...
			{
				int child1 = nodes[ancestor].Child1;
				int child2 = nodes[ancestor].Child2;

				// Update AABB of the ancestor to enclose its children:
				nodes[ancestor].Aabb = nodes[child1].Aabb.Merge(nodes[child2].Aabb);

				// Rotate the ancestor to maintain tree balance:
				Rotate(ancestor);

				// Move up to the next ancestor:
				ancestor = nodes[ancestor].Parent;
			}
		}
		else // If the removed leaf's parent was the root...
		{
			root = sibling; // The sibling becomes the new root.
			nodes[sibling].Parent = NULL_NODE; // The new root has no parent.
		}
	}

	/// <summary>
	/// Allocates a new node from the pool.
	/// </summary>
	/// <returns>The index of the newly allocated node.</returns>
	/// <remarks>
	/// This method manages the internal node pool, growing it if necessary.
	/// It uses a free list for efficient memory management.
	/// </remarks>
	private int AllocateNode()
	{
		if (freeList == NULL_NODE)
		{
			int oldCapacity = nodeCapacity;
			nodeCapacity += nodeCapacity / 2;
			Array.Resize(ref nodes, nodeCapacity);

			for (int i = oldCapacity; i < nodeCapacity - 1; i++)
			{
				nodes[i] = new Node();
				nodes[i].Next = i + 1;
				nodes[i].Parent = i;
			}
			nodes[nodeCapacity - 1] = new Node();
			nodes[nodeCapacity - 1].Next = NULL_NODE;
			nodes[nodeCapacity - 1].Parent = nodeCapacity - 1;

			freeList = oldCapacity;
		}

		int node = freeList;
		freeList = nodes[node].Next;
		nodes[node].Parent = NULL_NODE;
		nodes[node].Child1 = NULL_NODE;
		nodes[node].Child2 = NULL_NODE;
		nodes[node].Moved = false;
		nodeCount++;

		return node;
	}

	/// <summary>
	/// Frees a node, returning it to the pool.
	/// </summary>
	/// <param name="node">The index of the node to free.</param>
	/// <remarks>
	/// This method manages the internal node pool, adding the freed node
	/// to the free list for future reuse.
	/// </remarks>
	private void FreeNode(int node)
	{
		nodes[node].Next = freeList;
		nodes[node].Parent = node;
		nodes[node].Data = null!;
		freeList = node;
		nodeCount--;
	}

	/// <summary>
	/// Performs an AABB cast through the tree.
	/// </summary>
	/// <param name="input">The input parameters for the AABB cast.</param>
	/// <param name="callback">A callback function invoked for potential intersections.</param>
	/// <remarks>
	/// This method casts an AABB through the tree, finding potential intersections.
	/// It's useful for continuous collision detection or ray casting with volume.
	/// </remarks>
	public void AABBCast(AABBCastInput input, Func<AABBCastInput, TData, float> callback)
	{
		Vector3 p1 = input.From;
		Vector3 p2 = input.To;
		Vector3 r = input.HalfExtents;

		float maxFraction = input.MaxFraction;

		Vector3 d = p2 - p1;
		float length = d.Length();
		if (length <= 0.0f)
		{
			return;
		}

		Vector3 n = d / length;

		var stack = new Stack<int>();
		stack.Push(root);

		while (stack.Count > 0)
		{
			int nodeId = stack.Pop();
			if (nodeId == NULL_NODE)
			{
				continue;
			}

			Node node = nodes[nodeId];

			if (node.IsLeaf)
			{
				AABBCastInput subInput = new AABBCastInput
				{
					From = p1,
					To = p2,
					MaxFraction = maxFraction,
					HalfExtents = r
				};

				float newFraction = callback(subInput, node.Data);

				if (newFraction == 0.0f)
				{
					return;
				}

				if (newFraction > 0.0f && newFraction < maxFraction)
				{
					maxFraction = newFraction;
				}
			}
			else
			{
				int child1 = node.Child1;
				int child2 = node.Child2;

				float dist1 = nodes[child1].Aabb._RayCast(p1, p2, 0f, maxFraction, r);
				float dist2 = nodes[child2].Aabb._RayCast(p1, p2, 0f, maxFraction, r);

				if (dist2 < dist1)
				{
					(dist1, dist2) = (dist2, dist1);
					(child1, child2) = (child2, child1);
				}

				if (dist1 == float.MaxValue)
				{
					continue;
				}
				else
				{
					if (dist2 != float.MaxValue)
					{
						stack.Push(child2);
					}
					stack.Push(child1);
				}
			}
		}
	}

	/// <summary>
	/// Computes the total cost of the tree based on surface area heuristic.
	/// </summary>
	/// <returns>The total cost of the tree.</returns>
	/// <remarks>
	/// This method calculates the sum of all node surface areas in the tree.
	/// It's useful for evaluating the overall quality of the tree structure.
	/// </remarks>
	public float ComputeTreeCost()
	{
		float cost = 0.0f;

		for (int i = 0; i < nodeCapacity; i++)
		{
			if (nodes[i].Parent == i) continue;
			cost += SurfaceArea(nodes[i].Aabb);
		}

		return cost;
	}

	/// <summary>
	/// Rebuilds the entire tree to optimize its structure.
	/// </summary>
	/// <remarks>
	/// This method reconstructs the tree from scratch using the current set of objects.
	/// It's useful for periodically optimizing the tree structure, especially after
	/// many updates or when the tree quality has degraded.
	/// </remarks>
	public void Rebuild()
	{
		RebuildInternal();

		nodeIndexLookup.Clear();
		for (int i = 0; i < nodeCapacity; i++)
		{
			if (nodes[i].Parent != i && nodes[i].IsLeaf)
			{
				nodeIndexLookup.TryAdd(nodes[i].Data, i);
			}
		}
	}

	/// <summary>
	/// Internal method to rebuild the tree structure.
	/// </summary>
	/// <remarks>
	/// This method implements the actual tree rebuilding algorithm, using a bottom-up approach
	/// to construct an optimized tree structure based on the current set of leaf nodes.
	/// </remarks>
	public void RebuildInternal()
	{
		int[] nodeIndices = new int[nodeCount];
		int count = 0;

		// Collect all leaf nodes
		for (int i = 0; i < nodeCapacity; i++)
		{
			if (nodes[i].Parent == i) continue;
			if (nodes[i].IsLeaf)
			{
				nodes[i].Parent = NULL_NODE;
				nodeIndices[count++] = i;
			}
			else
			{
				FreeNode(i);
			}
		}

		while (count > 1)
		{
			float minCost = float.MaxValue;
			int minI = -1;
			int minJ = -1;

			// Find the cheapest pair to merge
			for (int i = 0; i < count; i++)
			{
				Aabb aabbI = nodes[nodeIndices[i]].Aabb;

				for (int j = i + 1; j < count; j++)
				{
					Aabb aabbJ = nodes[nodeIndices[j]].Aabb;
					Aabb combinedAabb = aabbI.Merge(aabbJ);
					float cost = SurfaceArea(combinedAabb);

					if (cost < minCost)
					{
						minCost = cost;
						minI = i;
						minJ = j;
					}
				}
			}

			int index1 = nodeIndices[minI];
			int index2 = nodeIndices[minJ];
			Node child1 = nodes[index1];
			Node child2 = nodes[index2];

			int parentIndex = AllocateNode();
			Node parent = nodes[parentIndex];

			parent.Child1 = index1;
			parent.Child2 = index2;
			parent.Aabb = child1.Aabb.Merge(child2.Aabb);
			parent.Parent = NULL_NODE;

			child1.Parent = parentIndex;
			child2.Parent = parentIndex;

			nodeIndices[minI] = parentIndex;
			nodeIndices[minJ] = nodeIndices[count - 1];
			count--;
		}

		root = nodeIndices[0];
		nodes[root].Parent = NULL_NODE;

		_AssertTreeOk();
	}

	/// <summary>
	/// Tests if two objects in the tree have overlapping AABBs.
	/// </summary>
	/// <param name="dataA">The first object to test.</param>
	/// <param name="dataB">The second object to test.</param>
	/// <returns>True if the AABBs of the objects overlap, false otherwise.</returns>
	/// <remarks>
	/// This method provides a quick way to check for potential collisions between two objects
	/// without traversing the entire tree.
	/// </remarks>
	public bool TestOverlap(TData dataA, TData dataB)
	{
		if (!nodeIndexLookup.TryGetValue(dataA, out int nodeA) || !nodeIndexLookup.TryGetValue(dataB, out int nodeB))
		{
			throw __.Throw("item not found in AabbTree");
		}

		return TestOverlapInternal(nodeA, nodeB);
	}

	/// <summary>
	/// Internal method to test overlap between two nodes.
	/// </summary>
	/// <param name="nodeA">The index of the first node.</param>
	/// <param name="nodeB">The index of the second node.</param>
	/// <returns>True if the AABBs of the nodes overlap, false otherwise.</returns>
	/// <remarks>
	/// This method performs the actual AABB intersection test between two nodes.
	/// </remarks>
	private bool TestOverlapInternal(int nodeA, int nodeB)
	{
		__.Throw(nodeA >= 0 && nodeA < nodeCapacity);
		__.Throw(nodeB >= 0 && nodeB < nodeCapacity);

		return nodes[nodeA].Aabb.Intersects(nodes[nodeB].Aabb);
	}

	/// <summary>
	/// Gets the AABB of a specific object in the tree.
	/// </summary>
	/// <param name="data">The object to get the AABB for.</param>
	/// <returns>The AABB of the specified object.</returns>
	/// <remarks>
	/// This method is useful for retrieving the current bounding box of an object
	/// without having to traverse the entire tree.
	/// </remarks>
	public Aabb GetAABB(TData data)
	{
		if (!nodeIndexLookup.TryGetValue(data, out int node))
		{
			throw __.Throw("item not found in AabbTree");
		}

		return GetAABBInternal(node);
	}

	/// <summary>
	/// Internal method to get the AABB of a specific node.
	/// </summary>
	/// <param name="node">The index of the node.</param>
	/// <returns>The AABB of the specified node.</returns>
	private Aabb GetAABBInternal(int node)
	{
		__.Throw(node >= 0 && node < nodeCapacity);

		return nodes[node].Aabb;
	}

	/// <summary>
	/// Clears the 'moved' flag for a specific object in the tree.
	/// </summary>
	/// <param name="data">The object to clear the 'moved' flag for.</param>
	/// <remarks>
	/// This method is used to reset the movement status of an object after processing its movement.
	/// </remarks>
	public void ClearMoved(TData data)
	{
		if (nodeIndexLookup.TryGetValue(data, out int node))
		{
			ClearMovedInternal(node);
		}
		else
		{
			throw __.Throw("item not found in AabbTree");
		}
	}

	/// <summary>
	/// Internal method to clear the 'moved' flag for a specific node.
	/// </summary>
	/// <param name="node">The index of the node.</param>
	private void ClearMovedInternal(int node)
	{
		__.Throw(node >= 0 && node < nodeCapacity);
		nodes[node].Moved = false;
	}

	/// <summary>
	/// Checks if a specific object in the tree has moved.
	/// </summary>
	/// <param name="data">The object to check.</param>
	/// <returns>True if the object has moved, false otherwise.</returns>
	/// <remarks>
	/// This method is used to determine if an object's position has changed since the last update.
	/// </remarks>
	public bool WasMoved(TData data)
	{
		if (!nodeIndexLookup.TryGetValue(data, out int node))
		{
			throw __.Throw("item not found in AabbTree");
		}

		return WasMovedInternal(node);
	}

	/// <summary>
	/// Internal method to check if a specific node has moved.
	/// </summary>
	/// <param name="node">The index of the node.</param>
	/// <returns>True if the node has moved, false otherwise.</returns>
	private bool WasMovedInternal(int node)
	{
		//if (node < 0 || node >= nodeCapacity)
		//	throw new ArgumentOutOfRangeException(nameof(node));
		__.Throw(node >= 0 && node < nodeCapacity);

		return nodes[node].Moved;
	}

	/// <summary>
	/// Gets the data associated with a specific node.
	/// </summary>
	/// <param name="node">The index of the node.</param>
	/// <returns>The data associated with the specified node.</returns>
	/// <remarks>
	/// This method is useful for retrieving the object associated with a node
	/// during tree traversal or query operations.
	/// </remarks>
	public TData GetData(int node)
	{
		//if (node < 0 || node >= nodeCapacity)
		//	throw new ArgumentOutOfRangeException(nameof(node));
		__.Throw(node >= 0 && node < nodeCapacity);

		return nodes[node].Data;
	}

	/// <summary>
	/// Resets the tree to its initial empty state.
	/// </summary>
	/// <remarks>
	/// This method clears all nodes and resets the tree structure. It's useful for
	/// completely clearing the tree without deallocating memory.
	/// </remarks>
	public void Reset()
	{
		ResetInternal();
		nodeIndexLookup.Clear();
	}

	/// <summary>
	/// Internal method to reset the tree structure.
	/// </summary>
	/// <remarks>
	/// This method handles the actual resetting of the tree, including
	/// reinitializing the free list and clearing all nodes.
	/// </remarks>
	private void ResetInternal()
	{
		root = NULL_NODE;
		nodeCount = 0;
		Array.Clear(nodes);

		for (int i = 0; i < nodeCapacity - 1; i++)
		{
			nodes[i] = new Node();
			nodes[i].Next = i + 1;
			nodes[i].Parent = i;
		}
		nodes[nodeCapacity - 1] = new Node();
		nodes[nodeCapacity - 1].Next = NULL_NODE;
		nodes[nodeCapacity - 1].Parent = nodeCapacity - 1;

		freeList = 0;

		_AssertTreeOk();
	}

	/// <summary>
	/// Performs a tree rotation operation to potentially improve the tree's structure and reduce overlap.
	/// </summary>
	/// <param name="node">The index of the node to rotate.</param>
	/// <remarks>
	/// This method implements the tree rotation algorithm, which is a key optimization
	/// technique for maintaining tree balance and minimizing the total surface area of AABBs.
	/// It considers different rotation possibilities and chooses the one that results in
	/// the smallest combined surface area.
	/// </remarks>
	private void Rotate(int node)
	{
		//_AssertTreeOk(); //rotation can occur when tree is in an invalid state
		// If the node is a leaf node (no children), there's nothing to rotate, so return.
		if (nodes[node].IsLeaf)
		{
			return;
		}

		// Get the indices of the node's children.
		int child1 = nodes[node].Child1;
		int child2 = nodes[node].Child2;

		// Initialize an array to store the cost differences for potential rotations.
		// Each element represents the change in surface area after a specific rotation.
		float[] costDiffs = new float[4];

		// Calculate cost differences if child1 is not a leaf node.
		// This involves calculating the surface area of the AABB formed by merging one of child1's grandchildren with child2,
		// and subtracting the original surface area of child1's AABB.
		if (!nodes[child1].IsLeaf)
		{
			float area1 = SurfaceArea(nodes[child1].Aabb);
			costDiffs[0] = SurfaceArea(nodes[nodes[child1].Child1].Aabb.Merge(nodes[child2].Aabb)) - area1;
			costDiffs[1] = SurfaceArea(nodes[nodes[child1].Child2].Aabb.Merge(nodes[child2].Aabb)) - area1;
		}

		// Calculate cost differences if child2 is not a leaf node.
		// Similar to the previous block, this calculates the surface area change after merging one of child2's grandchildren with child1.
		if (!nodes[child2].IsLeaf)
		{
			float area2 = SurfaceArea(nodes[child2].Aabb);
			costDiffs[2] = SurfaceArea(nodes[nodes[child2].Child1].Aabb.Merge(nodes[child1].Aabb)) - area2;
			costDiffs[3] = SurfaceArea(nodes[nodes[child2].Child2].Aabb.Merge(nodes[child1].Aabb)) - area2;
		}

		// Find the index of the rotation that results in the smallest cost difference (greatest reduction in surface area).
		int bestDiffIndex = 0;
		for (int i = 1; i < 4; i++)
		{
			if (costDiffs[i] < costDiffs[bestDiffIndex])
			{
				bestDiffIndex = i;
			}
		}

		// If the best rotation doesn't reduce the surface area, don't perform the rotation.
		if (costDiffs[bestDiffIndex] >= 0.0f)
		{
			return;
		}

		// Perform the rotation based on the bestDiffIndex.
		// Each case represents a different rotation scenario, involving swapping grandchildren and updating parent-child relationships.
		switch (bestDiffIndex)
		{
			case 0:
				nodes[nodes[child1].Child2].Parent = node;
				nodes[node].Child2 = nodes[child1].Child2;

				nodes[child1].Child2 = child2;
				nodes[child2].Parent = child1;

				nodes[child1].Aabb = nodes[nodes[child1].Child1].Aabb.Merge(nodes[nodes[child1].Child2].Aabb);
				break;
			case 1:
				nodes[nodes[child1].Child1].Parent = node;
				nodes[node].Child2 = nodes[child1].Child1;

				nodes[child1].Child1 = child2;
				nodes[child2].Parent = child1;

				nodes[child1].Aabb = nodes[nodes[child1].Child1].Aabb.Merge(nodes[nodes[child1].Child2].Aabb);
				break;
			case 2:
				nodes[nodes[child2].Child2].Parent = node;
				nodes[node].Child1 = nodes[child2].Child2;

				nodes[child2].Child2 = child1;
				nodes[child1].Parent = child2;

				nodes[child2].Aabb = nodes[nodes[child2].Child1].Aabb.Merge(nodes[nodes[child2].Child2].Aabb);
				break;
			case 3:
				nodes[nodes[child2].Child1].Parent = node;
				nodes[node].Child1 = nodes[child2].Child1;

				nodes[child2].Child1 = child1;
				nodes[child1].Parent = child2;

				nodes[child2].Aabb = nodes[nodes[child2].Child1].Aabb.Merge(nodes[nodes[child2].Child2].Aabb);
				break;
		}
	}

	[Obsolete("the inline swaps found in .Rotate() are more efficient, but this could be used instead.")]
	private void Swap(int node1, int node2)
	{
		int parent1 = nodes[node1].Parent;
		int parent2 = nodes[node2].Parent;

		if (parent1 == parent2)
		{
			if (nodes[parent1].Child1 == node1)
			{
				nodes[parent1].Child1 = node2;
				nodes[parent1].Child2 = node1;
			}
			else
			{
				nodes[parent1].Child1 = node1;
				nodes[parent1].Child2 = node2;
			}
			return;
		}

		if (nodes[parent1].Child1 == node1)
			nodes[parent1].Child1 = node2;
		else
			nodes[parent1].Child2 = node2;

		if (nodes[parent2].Child1 == node2)
			nodes[parent2].Child1 = node1;
		else
			nodes[parent2].Child2 = node1;

		nodes[node1].Parent = parent2;
		nodes[node2].Parent = parent1;
	}


	/// <summary>
	/// Traverses the tree and applies the given action to each node's data.
	/// </summary>
	/// <param name="callback">The action to apply to each node's data.</param>
	/// <remarks>
	/// This method provides a way to iterate over all objects in the tree without
	/// performing any spatial queries. It's useful for operations that need to be
	/// applied to all objects regardless of their position.
	/// </remarks>
	public void Traverse(Action<TData> callback)
	{
		if (root == NULL_NODE)
		{
			return;
		}

		var stack = new Stack<int>();
		stack.Push(root);

		while (stack.Count > 0)
		{
			int current = stack.Pop();

			if (!nodes[current].IsLeaf)
			{
				stack.Push(nodes[current].Child1);
				stack.Push(nodes[current].Child2);
			}

			if (nodes[current].Data != null)
			{
				callback(nodes[current].Data);
			}
		}
	}

	/// <summary>
	/// Represents the input parameters for an AABB cast operation.
	/// </summary>
	public struct AABBCastInput
	{
		/// <summary>
		/// The starting point of the cast.
		/// </summary>
		public Vector3 From;

		/// <summary>
		/// The end point of the cast.
		/// </summary>
		public Vector3 To;

		/// <summary>
		/// The half-extents of the AABB being cast.
		/// </summary>
		public Vector3 HalfExtents;

		/// <summary>
		/// The maximum fraction of the cast to check (0.0 to 1.0).
		/// </summary>
		public float MaxFraction;
	}

	/// <summary>
	/// Enumerates all nodes in the tree.
	/// </summary>
	/// <returns>An IEnumerable of all active nodes in the tree.</returns>
	/// <remarks>
	/// This method provides a way to iterate over all nodes in the tree,
	/// which can be useful for debugging or advanced tree operations.
	/// </remarks>
	public IEnumerable<Node> EnumerateNodes()
	{
		for (int i = 0; i < nodeCapacity; i++)
		{
			if (nodes[i].Parent != i)
			{
				yield return nodes[i];
			}
		}
	}
}


/// <summary>
/// debug helpers
/// </summary>
public partial class AABBTree<TData> where TData : class
{
	/// <summary>
	/// Asserts that a node is alive (not in the free list).
	/// </summary>
	/// <param name="index">The index of the node to check.</param>
	/// <param name="doNotIgnoreNullNode">If true, throws an assertion for NULL_NODE.</param>
	/// <remarks>
	/// This method is used in debug builds to ensure that operations are performed on valid nodes.
	/// It helps catch issues where operations might be attempted on freed or invalid nodes.
	/// </remarks>
	[Conditional("DEBUG")]
	private void _AssertAlive(int index, bool doNotIgnoreNullNode = false)
	{
		if (index == NULL_NODE)
		{
			if (doNotIgnoreNullNode is false)
			{
				return;
			}
			else
			{
				__.Assert("index is NullNode");
			}
		}

		var nodeObj = nodes[index];
		__.Assert(nodeObj.Parent != index);
	}

	/// <summary>
	/// Performs a comprehensive check of the tree's integrity.
	/// </summary>
	/// <param name="ignoreAabbConsistency">If true, skips checking AABB consistency.</param>
	/// <remarks>
	/// This method verifies various properties of the tree structure, including:
	/// - Parent-child relationships
	/// - AABB consistency (unless ignored)
	/// - Node count accuracy
	/// - Absence of cycles in the tree
	/// It's a powerful tool for detecting structural issues in the tree.
	/// </remarks>
	[Conditional("DEBUG")]
	public void _AssertTreeOk(bool ignoreAabbConsistency = false)
	{
		if (root == NULL_NODE)
		{
			// Empty tree is OK
			return;
		}

		// 1. Check that all parent-child relationships are consistent
		for (int i = 0; i < nodeCapacity; i++)
		{
			if (nodes[i].Parent == i) continue; // Skip free nodes

			if (nodes[i].IsLeaf)
			{
				// Leaf nodes should not have children
				__.Assert(nodes[i].Child1 == NULL_NODE);
				__.Assert(nodes[i].Child2 == NULL_NODE);
			}
			else
			{
				// Internal nodes should have valid children
				__.Assert(nodes[i].Child1 != NULL_NODE);
				__.Assert(nodes[i].Child2 != NULL_NODE);

				// Children should point back to the parent
				__.Assert(nodes[nodes[i].Child1].Parent == i);
				__.Assert(nodes[nodes[i].Child2].Parent == i);
			}

			// Parent should be a valid node (or NullNode for root)
			if (i == root)
			{
				// Root node should have NullNode as parent
				__.Assert(nodes[i].Parent == NULL_NODE);
			}
			else
			{
				__.Assert(nodes[i].Parent >= 0 && nodes[i].Parent < nodeCapacity);
			}
		}

		// 2. Check that AABBs are consistent with their children
		if (ignoreAabbConsistency is false)
		{
			_AssertAabbConsistent(root);
		}

		// 3. Check that the node count is correct
		int actualNodeCount = 0;
		for (int i = 0; i < nodeCapacity; i++)
		{
			if (nodes[i].Parent != i)
			{
				actualNodeCount++;
			}
		}
		__.Assert(actualNodeCount == nodeCount);

		// 4. Check for ring structures (cycle detection)
		for (int i = 0; i < nodeCapacity; i++)
		{
			if (nodes[i].Parent == i) continue; // Skip free nodes

			_AssertNoCycles(i);
		}
	}

	/// <summary>
	/// Checks for cycles in the tree starting from a given node.
	/// </summary>
	/// <param name="nodeIndex">The starting node index for cycle detection.</param>
	/// <remarks>
	/// This method uses a hash set to detect cycles in the tree structure.
	/// It's crucial for ensuring the tree doesn't contain any circular references,
	/// which would lead to infinite loops during traversal.
	/// </remarks>
	[Conditional("DEBUG")]
	private void _AssertNoCycles(int nodeIndex)
	{
		HashSet<int> visited = new HashSet<int>();
		int current = nodeIndex;

		while (nodes[current].Parent != NULL_NODE)
		{
			if (visited.Contains(current))
			{
				__.Assert(false, "Cycle detected in the tree starting from node: " + nodeIndex);
				return; // No need to continue if a cycle is found
			}

			visited.Add(current);
			current = nodes[current].Parent;
		}
	}

	/// <summary>
	/// Recursively checks AABB consistency for a node and its descendants.
	/// </summary>
	/// <param name="nodeIndex">The index of the node to check.</param>
	/// <remarks>
	/// This method ensures that:
	/// - Leaf node AABBs are consistent with their data
	/// - Internal node AABBs fully enclose their children's AABBs
	/// It's crucial for maintaining the spatial accuracy of the tree structure.
	/// </remarks>
	[Conditional("DEBUG")]
	private void _AssertAabbConsistent(int nodeIndex)
	{
		if (nodeIndex == NULL_NODE) return;

		if (nodes[nodeIndex].IsLeaf)
		{
			// Leaf node AABB should be consistent with its data
			__.Assert(nodes[nodeIndex].Data is not null);
			var aabbBufferedSize = nodes[nodeIndex].Aabb._Scaled(1.01f); //in case of rounding errors?
			__.Assert(aabbBufferedSize.Encloses(nodes[nodeIndex].ItemAabb));
		}
		else
		{
			// Internal node AABB should enclose its children's AABBs
			Aabb child1Aabb = nodes[nodes[nodeIndex].Child1].Aabb;
			Aabb child2Aabb = nodes[nodes[nodeIndex].Child2].Aabb;
			Aabb nodeAabb = nodes[nodeIndex].Aabb;

			var aabbBufferedSize = nodeAabb._Scaled(1.01f); //in case of rounding errors?
			__.Assert(aabbBufferedSize.Encloses(child1Aabb));
			__.Assert(aabbBufferedSize.Encloses(child2Aabb));

			// Recursively check children
			_AssertAabbConsistent(nodes[nodeIndex].Child1);
			_AssertAabbConsistent(nodes[nodeIndex].Child2);
		}
	}
}
