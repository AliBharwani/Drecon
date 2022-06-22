using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class KDTree
{
    public class Node
    {
        public Node left;
        public Node right;
        public float[] data;
        public Node(float[] _data)
        {
            data = _data;
        }
    }
    // k represents the num dimensions
    // extraData represents the num of floats at the end that should be ignored
    private int k;
    private int extraData;
    private List<float[]> values;
    private Node root;
    public KDTree (int _k = 30, int _extraData = 2)
    {
        k = _k;
        extraData = _extraData;
        values = new List<float[]>();
    }
   public void Add(float[] entry)
    {
        values.Add(entry);
    }


    public void Build()
    {
        root = recurs(0, values);
        values = null;
    }
    private Node recurs(int depth , List<float[]> values)
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
        newNode.left = recurs(depth + 1, valuesLeft);
        newNode.right = recurs(depth + 1, valuesRight);
        return newNode;
    }

    public float[] nnSearch(float[] searchVector )
    {
        return new float[0];
    }


}
