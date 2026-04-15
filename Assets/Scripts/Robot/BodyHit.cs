using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BodyHit : MonoBehaviour
{
    RobotWalker agent;

    private void Awake()
    {
        agent = GetComponentInParent<RobotWalker>();
    }
    private void OnCollisionStay(Collision collision)
    {
        agent.BodyHit(name, collision);
    }
}
