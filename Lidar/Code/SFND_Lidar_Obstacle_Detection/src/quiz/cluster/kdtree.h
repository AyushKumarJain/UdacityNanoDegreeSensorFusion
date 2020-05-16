
// Student Code Start
// 3D KdTree Implementation
/* \author Aaron Brown */
// Quiz on implementing kd tree
#ifndef KDTREE_H //These headers are included so that KdTree can be used in the Project final course.
#define KDTREE_H 

#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node {
    std::vector<float> point;
    int id;
    Node *left;
    Node *right;

    Node(std::vector<float> arr, int setId) : point(arr), id(setId), left(NULL), right(NULL) {
    }
};

struct KdTree {
  private:
    static constexpr int X {0}, Y {1}, Z {2}; // coord is used to choose a coordinate, it can be either X axis, Y axis or Z axis.

    bool withinBox(const std::vector<float> target, const Node* node, const float tolerance) 
	{

        const float leftX = target[X] - tolerance;
        const float rightX = target[X] + tolerance;

        const bool withinXbounds = node->point[X] >= leftX && node->point[X] <= rightX ;

        const float lowerY = target[Y] - tolerance;
        const float upperY = target[Y] + tolerance;

        const bool withinYbounds = node->point[Y] >= lowerY && node->point[Y] <= upperY;

        const float lowerZ = target[Z] - tolerance;
        const float upperZ = target[Z] + tolerance;

        const bool withinZbounds = node->point[Z] >= lowerZ && node->point[Z] <= upperZ;

        return (withinXbounds && withinYbounds && withinZbounds);
    }

    float withinRange(const std::vector<float> p1, const std::vector<float> p2, const float tolerance) 
	{
        const float xComponent {p1.at(X) - p2.at(X)};
        const float yComponent {p1.at(Y) - p2.at(Y)};
        const float zComponent {p1.at(Z) - p2.at(Z)};

        const float distance {std::sqrt((xComponent * xComponent) + (yComponent * yComponent) + (zComponent * zComponent))};

        return (distance <= tolerance);
    }
     
	// recursiveInsert is similar to insertHelper in the tutorials
    void recursiveInsert(Node** node, const int coord, const std::vector<float> point, const int id) 
	{

        if (*node == nullptr) {
            *node = new Node(point, id);
            return;
        }

        const bool goLeft{ point.at(coord) < (*node)->point.at(coord) };

        recursiveInsert((goLeft ? &(*node)->left : &(*node)->right), ((coord + 1) % 3), point, id);
    }
    
	// recursiveSearch is similar to searchHelper in the tutorials
    void recursiveSearch(const std::vector<float>& target, Node* node, const int coord, const float tolerance, std::vector<int>& ids) 
	{

        if (node == nullptr) 
			return; 
        // withinBox and withinRange are defined to make the code look clear and structured.
        if (withinBox(target, node, tolerance) && withinRange(node->point, target, tolerance))
            ids.push_back(node->id);

        if ((target.at(coord) - tolerance) < node->point.at(coord))
            recursiveSearch(target, node->left, ((coord + 1) % 3), tolerance, ids);

        if ((target.at(coord) + tolerance) > node->point.at(coord))
            recursiveSearch(target, node->right, ((coord + 1) % 3), tolerance, ids);
    }

  public:
    Node* root;

    KdTree() : root(NULL) {
    }

    void insert(const std::vector<float> point, int id) // const has been added compared to old code 
	{
        // TODO: Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the root

        recursiveInsert(&root, X, point, id); // X has been added compared to old code
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(const std::vector<float> target, const float distanceTol) // const has been added compared to old code 
	{
        Node *node {root}; // It has been added compared to old code

        std::vector<int> ids;
        recursiveSearch(target, root, X, distanceTol, ids); // X has been added compared to old code
        return ids;
    }
};

#endif

//Student Code End

// In order to run course tutorial KdTree uncommend the following code and comment out student code.
// Old Code Start
// 2D KdTree Implementation
/* \author Aaron Brown */
// Quiz on implementing kd tree

/* #include "../../render/render.h"

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left; // A binary nody, as it has left and right child
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root; // Node in the tree, root is a Node pointer

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id) // id will go from 0 to 10
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root,0,point,id); // memory address of root - &root


	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}

	void insertHelper(Node** node, uint depth, std::vector<float> point, int id) //Node** double pointer to Node is the memory address of the Node.
	{ 
		// Tree is empty
		// Here, node is a memory address, (&root also gives memory address), (Node** also used to create a memory address) but (Node* is used to create a pointer)
		// We could also have used Node*& node above. it is also called pointer reference (*&). It could also work, but then we would have needed to deference it also below at *node. 
		if(*node == NULL) // When we derefrence node by using *node, it gives us value of the memory address 
		{
			*node = new Node(point,id); // We deference our node by using *node and assign a brand new Node (its structure defined above as struct Node). It needs point value and the id 
		}
		else 
		{
			//Calculate current dim
			uint cd = depth % 2; // cd will be 0 or 1, unit is used for unsigned, we will not have negative depth only positive

			if(point[cd] < ((*node)->point[cd]))
			{
				insertHelper(&((*node)->left), depth+1, point, id); // & is for address, *node is to dereference
			}
			else
			{
				insertHelper(&((*node)->right), depth+1, point, id);
			}

		}

    // insertHelper will terminate when it is null node.
	}

    void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if(node!=NULL)
		{
			if ((node -> point[0] >= (target[0]-distanceTol) && node -> point[0] <= (target[0]+distanceTol))  && (node -> point[1] >= (target[1]-distanceTol) && node -> point[1] <= (target[1]+distanceTol)))
			{
				float distance = sqrt((node->point[0]-target[0])*(node->point[0]-target[0])+(node->point[1]-target[1])*(node->point[1]-target[1]));
				if (distance <= distanceTol)
					ids.push_back(node->id);
			}

			// Check accross boundary
            // target[depth%2] will check if x or y value is needed to be compared, which inturn is decided by depth value. We start from depth 0, root, with x comparison
			if ( (target[depth%2]-distanceTol) < node->point[depth%2])
				searchHelper(target,node->left,depth+1,distanceTol,ids);
			if ( (target[depth%2]+distanceTol) > node->point[depth%2])
				searchHelper(target,node->right,depth+1,distanceTol,ids);
				
		}
	}	

};

// Old Code End
 */



