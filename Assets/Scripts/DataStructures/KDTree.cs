using System.Collections.Generic;
using UnityEngine;

public class KDTree
{
    public class Node
    {
        public Node left;
        public Node right;
        public float[] data;
        public bool isLeaf;
        public Node(float[] _data)
        {
            data = _data;
        }
    }
    // k represents the num dimensions
    // extraData represents the num of floats at the end that should be ignored
    private int k;
    private int extraData;
    private int numNeigh;

    private List<float[]> values;
    private Node root;
    private float currentBestDist;
    private Node closest;
    private int ignore_surrounding;

    private MaxHeap<float> maxHeap;

    public KDTree(int _k, int _extraData = 2, int _numNeigh = 1, int _ignore_surrounding = 10)
    {
        k = _k;
        extraData = _extraData;
        numNeigh = _numNeigh;
        ignore_surrounding = _ignore_surrounding;
        if (numNeigh > 1)
            maxHeap = new MaxHeap<float>(numNeigh);
        values = new List<float[]>();
    }
   public void Add(float[] entry)
    {
        values.Add(entry);
    }

    public void Build()
    {
        root = recursiveBuild(0, values);
        values = null;
    }

    private Node recursiveBuild(int depth , List<float[]> values)
    {
        if (values.Count == 0)
            return null;
        int axis = depth % k;
        values.Sort(delegate (float[] x, float[] y)
        {
            return x[axis].CompareTo(y[axis]);
        });
        int median_idx = values.Count / 2;
        Node newNode = new Node(values[median_idx]);
        int numValuesAfterMedian = values.Count - (median_idx + 1);
        List<float[]> valuesLeft = values.GetRange(0, median_idx);
        List<float[]> valuesRight = values.GetRange(median_idx + 1, numValuesAfterMedian);
        newNode.left = recursiveBuild(depth + 1, valuesLeft);
        newNode.right = recursiveBuild(depth + 1, valuesRight);
        return newNode;
    }

    public float[] nnSearch(float[] searchVector, int depth = 0 )
    {
        closest = null;
        currentBestDist = float.PositiveInfinity;
        if (numNeigh > 1)
            maxHeap.reset();
        int frame_idx = (int)searchVector[searchVector.Length - 1];
        recursiveNNSearch(root, searchVector, depth, frame_idx);
        if (numNeigh == 1) {
            return closest.data;
        } else
        {
            float[] returnVal = maxHeap.getRandom();
            return returnVal;
        }
    }

    private void recursiveNNSearch(Node node, float[] searchVector, int depth, int frame_idx) 
    {
        if (node == null)
            return;
        int axis = depth % k;
        // if the bounding box is too far, do nothing 
        //if (closest != null && Math.Pow(node.data[axis] - searchVector[axis], 2) > currentBestDist)
        //    return;
        int node_frame_idx = (int) node.data[k + extraData - 1];
        float dist = distanceBetween( node.data, searchVector);
        bool ignore = Mathf.Abs(node_frame_idx - frame_idx) < ignore_surrounding;
        if (!ignore)
        {
            if (numNeigh == 1)
            {
                if (closest == null || dist < currentBestDist)
                {
                    closest = node;
                    currentBestDist = dist;
                }
            }
            else
            {
                // maxHeap of Mins
                if (!maxHeap.isFull() || maxHeap.peek() > dist)
                    maxHeap.add(dist, node.data);
            }
        }


        if (searchVector[axis] < node.data[axis])
        {
            // search left first
            recursiveNNSearch(node.left, searchVector, depth + 1, frame_idx);
            // do the weird hypersphere thing wikipedia talked about 
            // ok but really: 
            // If the distance between the node's value at the splitting dimension and the search vector's va
            if (distBetweenAtAxis(node.data, searchVector, axis) < currentBestDist)
            {
                recursiveNNSearch(node.right, searchVector, depth + 1, frame_idx);
            }
        } else
        {
            // search right first
            recursiveNNSearch(node.right, searchVector, depth + 1, frame_idx);
            if (distBetweenAtAxis(node.data, searchVector, axis) < currentBestDist)
            {
                recursiveNNSearch(node.left, searchVector, depth + 1, frame_idx);
            }
        }
    }
    private float distBetweenAtAxis(float[] a, float[] b, int axis)
    {
        return Mathf.Pow(a[axis] - b[axis], 2);
    }
    private float distanceBetween(float[] a, float[] b)
    {
        // use squared euclidan distance to avoid having to calculate square roots
        float answer = 0;
        for (int i = 0; i < k; i++)
        {
            answer += Mathf.Pow(a[i] - b[i], 2);
        }
        return  answer;
        //return Math.Sqrt(answer);
    }

    public float[] bruteForceSearch(float[] searchVector)
    {
        closest = null;
        currentBestDist = float.PositiveInfinity;

        recursiveBruteForceSearch(root, searchVector);
        return closest.data;
    }
    private void recursiveBruteForceSearch(Node node, float[] searchVector)
    {
        if (node == null)
            return;
        float dist = distanceBetween( node.data, searchVector);
        if (closest == null || dist < currentBestDist)
        {
            closest = node;
            currentBestDist = dist;
        }
        recursiveBruteForceSearch(node.left , searchVector);
        recursiveBruteForceSearch(node.right, searchVector);
    }
}
