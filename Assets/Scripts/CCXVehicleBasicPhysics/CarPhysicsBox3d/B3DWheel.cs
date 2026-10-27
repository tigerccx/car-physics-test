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

        private void Awake()
        {
            //Assign a typical inertia
            if(m_wheelProperties.m_inertia < 0.0f)
            {
                m_wheelProperties.m_inertia = 0.5f * m_wheelProperties.m_mass * 
                    m_wheelProperties.m_wheelRadius * m_wheelProperties.m_wheelRadius;
            }
        }
    }
}
