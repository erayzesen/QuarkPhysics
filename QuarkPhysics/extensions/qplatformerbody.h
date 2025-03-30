
/************************************************************************************
 * MIT License
 *
 * Copyright (c) 2023 Eray Zesen
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * https://github.com/erayzesen/QuarkPhysics
 *
**************************************************************************************/

#ifndef QPLATFORMERBODY
#define QPLATFORMERBODY

#include "../qrigidbody.h"
#include "../qcollision.h"

/**
 * @brief QPlatformerBody provides a ready-to-use foundation for character physics in platformer games.
 * It includes behaviors such as gravity, walking on slopes, and jumping. 
 * Additionally, it offers helper methods and properties for further customization.
 */

class QPlatformerBody : public QRigidBody{
protected:
       

    bool onFloor=false;
    bool onCeiling=false;


    //Platform Collider Bit Mask

    int platformLayersBit=0;

    //Floor
    float maxFloorAngle=M_PI*0.25;

    //Movable Floors
    float movingFloorSnapOffset=10.0f;
    QRigidBody *lastMovableFloor=nullptr;


    

    //Gravity

    QVector gravity=QVector(0,0.3);

    float gravityMultiplier=1.0f;

    QVector velocity=QVector::Zero();

    //Directions According to the Gravity
    QVector upDirection=QVector::Up();
    QVector rightDirection=QVector::Right();

    //Walk
    float walkSpeed=3.0;

    //Controller Velocities
    QVector horizontalVelocity=QVector::Zero();
    QVector verticalVelocity=QVector::Zero();
    bool isFalling=false;
    bool isRising=false;

    int walkSide=0;

    float walkAccelerationRate=0.1f;
    float walkDecelerationRate=0.1f;

    //Jump
    enum JumpModes{
        RELEASED,
        PRESSED,
        PRESSING,
    };
    enum JumpModes jumpMode=JumpModes::RELEASED;
    float jumpForce=5.0f;
    int maxJumpCount=2;
    int currentJumpCount=0;
    int jumpDurationFrameCount=30;
    int jumpFrameCountDown=0;
    float jumpGravityMultiplier=0.4f;
    float jumpFallGravityMultiplier=1.0f;

    

    virtual void PostUpdate();
    
    
public:

    struct CollisionTestInfo{
        QBody* body;
        QVector position;
        float penetration;
        QVector normal;
        CollisionTestInfo(QBody *body=nullptr,QVector position=QVector::Zero(),float penetration=0.0f,QVector normal=QVector::Zero() ): body(body), position(position),penetration(penetration),normal(normal) {}
    };
    
    QPlatformerBody();

    

    //Floor

    /**
     * @brief Sets the snap offset for moving floors.
     * @param value The snap offset value.
     * @return A pointer to the body itself.
     */
    QPlatformerBody * SetMovingFloorSnapOffset(float value);

    /**
     * @brief Gets the snap offset for moving floors.
     * @return The snap offset value.
     */
    float GetMovingFloorSnapOffset();

    /**
     * @brief Sets the maximum angle for the floor.
     * @param value The maximum angle in radians.
     * @return A pointer to the body itself.
     */
    QPlatformerBody * SetFloorMaxAngle(float value);
    /**
     * @brief Gets the maximum angle for the floor.
     * @return The maximum angle in radians.
     */
    float GetFloorMaxAngle();

    /**
     * @brief Sets the maximum angle for the floor in degrees.
     * @param value The maximum angle in degrees.
     * @return A pointer to the body itself.
     */
    QPlatformerBody * SetFloorMaxAngleDegree(float value);

    /**
     * @brief Gets the maximum angle for the floor in degrees.
     * @return The maximum angle in degrees.
     */
    float GetFloorMaxAngleDegree();

    /**
     * @brief Checks if the body is currently on the floor.
     * @return True if the body is on the floor, false otherwise.
     */
    bool GetIsOnFloor();
    /**
     * @brief Checks if the body is currently touching the ceiling.
     * @return True if the body is on the ceiling, false otherwise.
     */
    bool GetIsOnCeiling();
    

    //Gravity
    /**
     * @brief Sets the gravity vector.
     * @param value The gravity vector.
     * @return A pointer to the body itself.
     */
    QPlatformerBody * SetGravity(QVector value);

    /**
     * @brief Gets the gravity vector.
     * @return The gravity vector.
     */
    QVector GetGravity();

    /**
     * @brief Sets the gravity multiplier.
     * @param value The gravity multiplier.
     * @return A pointer to the body itself.
     */
    QPlatformerBody * SetGravityMultiplier(float value);
    /**
     * @brief Gets the gravity multiplier.
     * @return The gravity multiplier.
     */
    float GetGravityMultiplier();
    
    //Walk
    /**
     * @brief Sets the walking speed of the body.
     * @param value The walking speed.
     * @return A pointer to the body itself.
     */
    QPlatformerBody * SetWalkSpeed(float value);

    /**
     * @brief Gets the walking speed of the body.
     * @return The walking speed
     */
    float GetWalkSpeed();

    /**
     * @brief Sets the walking acceleration rate.
     * @param value The acceleration rate.
     * @return A pointer to the body itself.
     */
    QPlatformerBody * SetWalkAcelerationRate(float value);

    /**
     * @brief Gets the walking acceleration rate.
     * @return The acceleration rate.
     */
    float GetWalkAcelerationRate();

    /**
     * @brief Sets the walking deceleration rate.
     * @param value The deceleration rate.
     * @return A pointer to the body itself.
     */
    QPlatformerBody * SetWalkDecelerationRate(float value);

    /**
     * @brief Gets the walking deceleration rate.
     * @return The deceleration rate.
     */
    float GetWalkDecelerationRate();

    /**
     * @brief Moves the body in a specified horizontal direction.
     * @param side The direction (-1 for left, 1 for right, 0 for no movement).
     * @return A pointer to the body itself.
     */
    QPlatformerBody * Walk(int side);

    //Controller Velocities

    /**
     * @brief Sets the horizontal velocity controlled by the character physics.
     * @param value The horizontal velocity vector.
     * @return A pointer to the body itself.
     */
    QPlatformerBody * SetControllerHorizontalVelocity(QVector value);

    /**
     * @brief Gets the horizontal velocity controlled by the character physics.
     * @return The horizontal velocity vector.
     */
    QVector GetControllerHorizontalVelocity();

    /**
     * @brief Sets the vertical velocity controlled by the character physics.
     * @param value The vertical velocity vector.
     * @return A pointer to the body itself.
     */
    QPlatformerBody * SetControllerVerticalVelocity(QVector value);

    /**
     * @brief Gets the vertical velocity controlled by the character physics.
     * @return The vertical velocity vector.
     */
    QVector GetControllerVerticalVelocity();

    /**
     * @brief Checks if the body is falling.
     * @return True if the body is falling, false otherwise.
     */
    bool GetIsFalling();

    /**
     * @brief Checks if the body is rising.
     * @return True if the body is rising, false otherwise.
     */
    bool GetIsRising();

    //Jump
    /**
     * @brief Sets the duration for a jump in frames.
     * @param value The jump duration in frames.
     * @return A pointer to the body itself.
     */
    QPlatformerBody * SetJumpDurationFrameCount(int value);

    /**
     * @brief Gets the duration for a jump in frames.
     * @return The jump duration in frames.
     */
    int GetJumpDurationFrameCount();

    /**
     * @brief Sets the gravity multiplier during a jump.
     * @param value The gravity multiplier.
     * @return A pointer to the body itself.
     */
    QPlatformerBody * SetJumpGravityMultiplier(float value);

    /**
     * @brief Gets the gravity multiplier during a jump.
     * @return The gravity multiplier.
     */
    float GetJumpGravityMultiplier();

    /**
     * @brief Sets the gravity multiplier for the falling phase of a jump.
     * @param value The gravity multiplier.
     * @return A pointer to the body itself.
     */
    QPlatformerBody * SetJumpFallGravityMultiplier(float value);

    /**
     * @brief Gets the gravity multiplier for the falling phase of a jump.
     * @return The gravity multiplier.
     */
    float GetJumpFallGravityMultiplier();

    /**
     * @brief Sets the maximum number of jumps allowed.
     * @param value The maximum jump count.
     * @return A pointer to the body itself.
     */
    QPlatformerBody * SetMaxJumpCount(int value);

    /**
     * @brief Gets the maximum number of jumps allowed.
     * @return The maximum jump count.
     */
    int GetMaxJumpCount();

    /**
     * @brief Initiates a jump with the specified force.
     * @param force The force of the jump.
     * @param unconditional If true, the jump will execute regardless of current conditions.
     * @return A pointer to the body itself.
     */
    QPlatformerBody * Jump(float force,bool unconditional=false);

    /**
     * @brief Releases the jump phase.
     * @return A pointer to the body itself.
     */
    QPlatformerBody * ReleaseJump();

    /**
     * @brief Checks if the body is currently jumping.
     * @return True if the body is jumping, false otherwise.
     */
    bool GetIsJumping();
    /**
     * @brief Checks if the jump phase has been released.
     * @return True if the jump phase is released, false otherwise.
     */
    bool GetIsJumpReleased();

    //Platforms
    /**
     * @brief Sets specific platform layers bit. 
     * This determines which platform layers the platformer physics will apply to, such as sloped surfaces, 
     * ceilings, walls, etc. The default value is 0, which disables this functionality. 
     * @note The object can still interact with objects in the physics world that are set to the layers specified for collision, but platformer-specific behavior (like gravity and movement on slopes) 
     * will be applied only to the specified platform layers. 
     * @param layersBit The bitmask representing the platform layers.
     * @return A pointer to the body itself.
     */
    QPlatformerBody * SetSpecificPlatformLayers(int layersBit); 

    /**
     * @brief Gets specific platform layers bit.
     * This determines which platform layers the platformer physics will apply to, such as sloped surfaces, 
     * ceilings, walls, etc. The default value is 0, which disables this functionality. 
     * @note The object can still interact with objects in the physics world that are set to the layers specified for collision, but platformer-specific behavior (like gravity and movement on slopes) 
     * will be applied only to the specified platform layers. 
     * @return The bitmask representing the platform layers.
     */
    int  GetSpecificPlatformLayers(); 

    /**
     * @brief Performs a platform collision test.
     * @param testPosition The position to test for collisions.
     * @param FilterByMovingDirection If set to true, filters out collisions that respond in the opposite direction of the movement vector calculated between the current position and the given test position.
     * @return A CollisionTestInfo structure with the collision details.
     */
    QPlatformerBody::CollisionTestInfo GetPlatformCollisions(QVector testPosition, bool FilterByMovingDirection=false );

    /**
     * @brief Checks for a collision with a wall on the right.
     * @param offset The offset to apply during the check.
     * @return A CollisionTestInfo structure with the collision details.
     */
    QPlatformerBody::CollisionTestInfo GetRightWall(float offset);

    /**
     * @brief Checks for a collision with a wall on the left.
     * @param offset The offset to apply during the check.
     * @return A CollisionTestInfo structure with the collision details.
     */
    QPlatformerBody::CollisionTestInfo GetLeftWall(float offset);

    /**
     * @brief Checks for a collision with the floor.
     * @param offset The offset to apply during the check.
     * @return A CollisionTestInfo structure with the collision details.
     */
    QPlatformerBody::CollisionTestInfo GetFloor(float offset);

    /**
     * @brief Checks for a collision with the ceiling.
     * @param offset The offset to apply during the check.
     * @return A CollisionTestInfo structure with the collision details.
     */
    QPlatformerBody::CollisionTestInfo GetCeiling(float offset);

    /** A force is applied to the platformer body. You can use the method safely before the physics step (e.g. at the OnPreStep event). If you want to use this method after physics step, it can break the simulation.(Collisions and constraints may not be applied properly.) if you want to apply force at the next physic step safely, use SetForce() and AddForce() methods.  
	 * @param force The force to apply.
	 */
    virtual QPlatformerBody * ApplyForce(QVector value) override;

    


    
    

    

};

#endif //QPLATFORMERBODY