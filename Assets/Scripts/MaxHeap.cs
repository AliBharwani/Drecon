using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

public class MaxHeap <T>
{
    public Tuple<float, T[]>[] arr;
    private int capacity;
    public int size;
    private Tuple<float, T[]> dummy = new Tuple<float, T[]>(-1, null);
   public MaxHeap(int _size )
    {
        size = 0;
        capacity = _size;
        arr = new Tuple<float, T[]>[_size + 1];
        reset();
    }

    public bool isFull()
    {
        return size == capacity;
    }
    public void reset()
    {
        for (int i = 0; i <= capacity; i++)
            arr[i] = dummy;
        size = 0;
    }

    public T[] getRandom()
    {
        return arr[UnityEngine.Random.Range(1, capacity + 1)].Item2;
    }

    public float peek()
    {
        if (size == 0)
        {
            return float.PositiveInfinity;
        }
        return arr[1].Item1;
    }
    public bool isGreater(int idxA, int idxB)
    {
        return arr[idxA].Item1 > arr[idxB].Item1;
    }

    public bool isGreaterThanParent(int idx)
    {
        if (idx == 1)
            return false;
        else
            return isGreater(idx, idx / 2);
    }

    public void swapWithParent(int idx)
    {
        if (idx == 1)
            return;
        int parentIdx = idx / 2;
        swap(idx, parentIdx);
    }

    public void swap(int a, int b)
    {
        var temp = arr[a];
        arr[a] = arr[b];
        arr[b] = temp;
    }
    public void heapify(int idx)
    {
        int largest = idx;
        int left = 2 * idx;
        int right = 2 * idx + 1;
        if (left < capacity + 1 && isGreater(left, idx))
            largest = left;
        if (right < capacity + 1 && isGreater(right, largest))
            largest = right;
        if (largest != idx)
        {
            swap(idx, largest);
            heapify(largest);
        }

    }
    public void add(float value, T[] data)
    {
        var entry = new Tuple<float, T[]>(value, data);
        if (size == capacity)
        {
            arr[1] = entry;
            for (int i = size / 2; i > 0; i--)
                heapify(i);
        } else
        {
            // add it to the back and keep bubbling it up
            int idx = size + 1;
            arr[idx] = entry;
            while (isGreaterThanParent(idx))
            {
                swapWithParent(idx);
                idx /= 2;
            }
            size++;
        }
    }
}
