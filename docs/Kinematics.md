## Structural Analysis of the Mechanism

To tinker with:
[parallel-arm-3d - GeoGebra](https://www.geogebra.org/classic/p4tu7z9s)

The manipulator analyzed here is a variant of the **Five-Bar Linkage** — a mechanism built from five links connected by five rotational joints.

Points **A** and **E** are fixed supports anchored at a distance _l₅_ from each other. Links _l₁_ and _l₄_ are the **input (driving) links**, each powered by its own motor at points A and E respectively. Links _l₂_ and _l₃_ are the **connecting rods**, meeting at point C. Link _l₆_ is simply an extension of _l₃_ beyond point D, ending at the **end-effector F** — the working tip of the manipulator.

**Quick summary of the geometry:**

-   _l₁_ and _l₄_ rotate around fixed points A and E
-   _l₂_ and _l₃_ connect at points B, D and meet each other at C
-   The segment C→F is one continuous link, split by joint D into _l₃_ and _l₆_
-   Every joint is a pure revolute (rotation only)

### How many degrees of freedom does it have?

Using the **Grübler–Kutzbach criterion**:

> DOF = 3(n − 1) − 2p₅ − p₄

With 5 moving links and 5 revolute joints:

> DOF = 3(5 − 1) − 2×5 = **2**

So the planar mechanism itself has 2 DOF — it needs two motors to be fully controlled. The full manipulator sits on a rotating platform, which adds a third degree of freedom, bringing the total to **3 DOF**.

----------

## Kinematic Layout

The mechanism uses the following link lengths:

| Parameter | Value | Description |
|-----------|-------|-------------|
| *l₀* | 135 mm | Platform height |
| *l₁* | 40 mm | Link AB |
| *l₂* | 160 mm | Link BC |
| *l₃* | 40 mm | Link CD |
| *l₄* | 160 mm | Link DE |
| *l₅* | 0 mm | Distance A→E |
| *l₆* | 160 mm | Link DF (end-effector arm) |

Angle **θ₁** rotates the whole mechanism around the vertical axis. Angles **θ₂** and **θ₅** define the positions of the two driving links relative to the x-axis.

---

## Forward Kinematics — Where does the tip end up?

Given the joint angles θ₁, θ₂, θ₅, we want to find the position of point F(x, y, z).

Starting from point A as the origin, we work our way through the chain:

-   **B:** `xB = l₁·cos θ₂`, `zB = l₁·sin θ₂`
-   **D:** `xD = l₅ + l₄·cos θ₅`, `zD = l₄·sin θ₅`

Thanks to the parallelogram structure (_l₁ || l₃_, _l₂ || l₄_), the projection of EF onto the horizontal plane simplifies neatly:

> L = l₄·cos θ₅ − l₆·cos θ₂

The 3D coordinates of the end-effector F are then:

> x = L·cos θ₁  
> y = L·sin θ₁  
> z = l₀ + l₄·sin θ₅ − l₆·sin θ₂

----------
