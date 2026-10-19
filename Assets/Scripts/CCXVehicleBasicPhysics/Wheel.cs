using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CCXUnityVehicle
{
    public class Wheel : MonoBehaviour
    {
        public WheelProperties m_wheelProperties;
        [ReadOnly]
        public WheelState m_wheelState;
    }
}