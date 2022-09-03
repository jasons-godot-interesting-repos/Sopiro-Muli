#pragma once

#include "common.h"
#include "rigidbody.h"

namespace spe
{

struct Settings;

class Constraint
{
public:
    Constraint(RigidBody* _bodyA, RigidBody* _bodyB, const Settings& _settings);
    virtual ~Constraint() noexcept = default;

    Constraint(const Constraint&) noexcept = delete;
    Constraint& operator=(const Constraint&) noexcept = delete;

    Constraint(Constraint&&) noexcept = default;
    Constraint& operator=(Constraint&&) noexcept = default;

    /*
     * C: Constraint equation
     * C = J·v = 0
     * J is depend on constraint
     *
     * Compute Jacobian J and effective mass M
     * M = K^-1 = (J · M^-1 · J^t)^-1
     */
    virtual void Prepare() = 0;

    /*
     * Solve velocity constraint, calculate corrective impulse for current iteration
     * Pc: Corrective impulse
     * λ: lagrangian multiplier
     *
     * Pc = J^t · λ (∵ Pc ∥ J^t)
     * λ = (J · M^-1 · J^t)^-1 ⋅ -(Jv + (β/h)·C(x)) where C(x): positional error
     *
     * with soft constraint,
     * λ = (J · M^-1 · J^t + λ·I)^-1 ⋅ -( Jv + (β/h)·C(x) + (γ/h)·λ' ) where I = identity matrix and λ' = accumulated impulse
     *
     * More reading:
     * https://pybullet.org/Bullet/phpBB3/viewtopic.php?f=4&t=1354
     */
    virtual void Solve() = 0;

    RigidBody* GetBodyA() const;
    RigidBody* GetBodyB() const;

protected:
    RigidBody* bodyA;
    RigidBody* bodyB;
    const Settings& settings;

    float beta{ 0.0f };
    float gamma{ 0.0f };
};

inline Constraint::Constraint(RigidBody* _bodyA, RigidBody* _bodyB, const Settings& _settings)
    : bodyA{ _bodyA }
    , bodyB{ _bodyB }
    , settings{ _settings }
{
}

inline RigidBody* Constraint::GetBodyA() const
{
    return bodyA;
}

inline RigidBody* Constraint::GetBodyB() const
{
    return bodyB;
}

} // namespace spe