using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public interface IClickable
{
    public virtual void OnEnterClicked() { }
    public virtual void OnHoldClicked() { }
    public virtual void OnExitClicked() { }

}
