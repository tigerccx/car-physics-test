using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(WheelCollider))]
public class BasicWheel : MonoBehaviour
{
    [Header("Basic Wheel Properties")]
    public float m_loadCoeff = 1.0f;

    public WheelCollider m_wheel;

    void Awake()
    {
        m_wheel = GetComponent<WheelCollider>();
    }
}
