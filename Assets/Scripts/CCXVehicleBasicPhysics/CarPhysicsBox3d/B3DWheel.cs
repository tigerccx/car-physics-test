using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace B3DVehicle
{
    public class Wheel : MonoBehaviour
    {
        public WheelProperties m_wheelProperties;
        [ReadOnly]
        public WheelState m_wheelState;
    }
}
