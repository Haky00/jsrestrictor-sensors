/** \file
 * \brief Library of functions for the Generic Sensor API wrappers
 *
 * \see https://www.w3.org/TR/generic-sensor/
 *
 *  \author Copyright (C) 2021  Radek Hranicky, 2024  Marek Hak
 *
 *  \license SPDX-License-Identifier: GPL-3.0-or-later
 */
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
/** \file
 * \ingroup wrappers
 *
 * Supporting fuctions for Generic Sensor API Wrappers
 */

/**
 * Contains a string which defines a variable called
 * skeleton_parameters_string that contains a JSON containing array
 * of skeleton parameters obtained from skeleton_parameters.json file
 */
var skeleton_parameters = `skeleton_parameters_string = \`${skeleton_parameters_json}\`;`;

/*
 * Functions for generating pseudorandom numbers.
 * To make the behavior deterministic and same on the same domain,
 * the generator uses domain hash as a seed.
 */
var sensorapi_prng_functions = `
  // Generates a 32-bit from a string. Inspired by MurmurHash3 algorithm
  // See: https://github.com/aappleby/smhasher/blob/master/src/MurmurHash3.cpp
  function sen_generateSeed(s) {
    var h;
    for(var i = 0, h = 1779033703 ^ s.length; i < s.length; i++)
      h = Math.imul(h ^ s.charCodeAt(i), 3432918353),
      h = h << 13 | h >>> 19;
    return h;
  }

  // Get seed for PRNG: prefer existing seed, then domain hash, session hash
  var sen_seed = sen_seed ||
    sen_generateSeed(domainHash);


  // PRNG based on Mulberry32 algorithm
  // See: https://gist.github.com/tommyettinger/46a874533244883189143505d203312c
	// To the extent possible under law, the author has dedicated all copyright
	// and related and neighboring rights to this software to the public domain
	// worldwide.
  function sen_prng() {
    // expects "seed" variable to be a 32-bit value
    var t = sen_seed += 0x6D2B79F5;
    t = Math.imul(t ^ t >>> 15, t | 1);
    t ^= t + Math.imul(t ^ t >>> 7, t | 61);
    return ((t ^ t >>> 14) >>> 0) / 4294967296;
  }

  // Generates a number around the input number
  function sen_generateAround(number, tolerance) {
    let min = number - number * tolerance;
    let max = number + number * tolerance;

    return sen_prng() * (max - min) + min;
  }

  // Generates a random number in the given interval
  function sen_generateMinMax(min, max) {
    return sen_prng() * (max - min) + min;
  }

  // Rounds a number to a fixed amount of decimal places
  // Returns a NUMBER
  function fixedNumber(num, digits, base) {
    var pow = Math.pow(base||10, digits);
    return Math.round(num*pow) / pow;
  }
`;

// Class providing the same pseudorandom number generation as the sen_prng() functions,
// but with a distinct seed value for each instance
class SenPseudoRandom {
  constructor() {
    this.seed = null;
  }

  // PRNG based on Mulberry32 algorithm
  // See: https://gist.github.com/tommyettinger/46a874533244883189143505d203312c
  // To the extent possible under law, the author has dedicated all copyright
  // and related and neighboring rights to this software to the public domain
  // worldwide.
  // Generates a number between 0 and 1, based on seed
  next() {
    if (this.seed == null) {
      this.generateSeed(domainHash);
    }
    // Expects "seed" variable to be a 32-bit value
    var t = (this.seed += 0x6d2b79f5);
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  }

  // Generates a number around the input number
  nextAround(number, tolerance) {
    let min = number - number * tolerance;
    let max = number + number * tolerance;

    return this.next() * (max - min) + min;
  }

  // Generates a random number in the given interval
  nextMinMax(min, max) {
    return this.next() * (max - min) + min;
  }

  // Generates a 32-bit from a string. Inspired by MurmurHash3 algorithm
  // See: https://github.com/aappleby/smhasher/blob/master/src/MurmurHash3.cpp
  generateSeed(s) {
    var h;
    for (var i = 0, h = 1779033703 ^ s.length; i < s.length; i++)
      (h = Math.imul(h ^ s.charCodeAt(i), 3432918353)),
        (h = (h << 13) | (h >>> 19));
    this.seed = h;
  }
}

/*
 * Functions for simulation of the device orientation.
 * Those allow to create a fake orientation of the device in axis angles
 * and create a rotation matrix. Support for multiplication of a 3D vector
 * with the rotation matrix is included.
 *
 * Note: The code needs supporting function from the
 * "sensorapi_prng_functions" above.
 *
 * In case of a non-rotated phone with a display oriented directly to the
 * face of the user, the device's axes are oriented as follows:
 *   x-axis is oriented from the user's left to the right
 *   y-axis from the bottom side of the display towards the top side
 *   z-axis is perpendicular to the display, it leads from the phone's
 *          display towards the user's face
 *
 * The yaw, pitch, and roll define the rotation of the phone in the Earth's
 * reference coordinate system. In case, all are 0:
 *   x is oriented towards the EAST
 *   y is oriented towards the NORTH (Earth's magnetic)
 *  -z is oriented toward the center of the Earth
 *
 *                   y (roll)
 *                  /  (NORTH if yaw = pitch = 0)
 *                 /
 *          +----------+
 *         /     /    /
 *  (top) / z(yaw)   /
 *       /   |/     /
 *      /    +-----/----> x (pitch)
 *     /          /      (EAST if yaw = roll = 0)
 *    /   _ _    /
 *   /   /__/   /
 *  +----------+
 *  (bottom)
 *
 */
var device_orientation_functions = `
  // Calcultes a rotation matrix for the given yaw, pitch, and roll
  // (in radians) of the device.
  function calculateRotationMatrix(yaw, pitch, roll) {
    var rotMat = [
      [Math.cos(yaw) * Math.cos(pitch),
       Math.cos(yaw) * Math.sin(pitch) * Math.sin(roll) - Math.sin(yaw) * Math.cos(roll),
       Math.cos(yaw) * Math.sin(pitch) * Math.cos(roll) + Math.sin(yaw) * Math.sin(roll)
      ],
      [Math.sin(yaw) * Math.cos(pitch),
       Math.sin(yaw) * Math.sin(pitch) * Math.sin(roll) + Math.cos(yaw) * Math.cos(roll),
       Math.sin(yaw) * Math.sin(pitch) * Math.cos(roll) - Math.cos(yaw) * Math.sin(roll)
      ],
      [(-1) * Math.sin(pitch),
       Math.cos(pitch) * Math.sin(roll),
       Math.cos(pitch) * Math.cos(roll)
      ]
    ];
    return rotMat;
  }

  // Initial draw of the (fake) device orientation
  // TODO: Limit to oriententations that make sense for a mobile device
  function generateDeviceOrientation() {
    var orient = {};
    /*
     * Yaw (couterclockwise rotation of the Z-axis)
     * Pitch (counterclockwise rotation of the Y-axis)
     * Roll (counterclockwise rotation of the X-axis)
     */
    var yaw = Math.floor(sen_prng() * 2 * Math.PI);
    var pitch = Math.floor(sen_prng() * 2 * Math.PI);
    var roll = Math.floor(sen_prng() * 2 * Math.PI);

    orient.yaw = yaw;
    orient.pitch = pitch;
    orient.roll = roll;

    // Calculate the rotation matrix
    orient.rotMat = calculateRotationMatrix(yaw, pitch, roll);

    return orient;
  }

  var orient = orient || generateDeviceOrientation();

  // Multiplies a 3D strength vector (1x3) with a 3D rotation matrix (3x3)
  // Returns the resulting 3D vector (1x3)
  function multVectRot(vec, mat) {
    var result = [
      vec[0]*mat[0][0] + vec[1]*mat[0][1] + vec[2]*mat[0][2],
      vec[0]*mat[1][0] + vec[1]*mat[1][1] + vec[2]*mat[1][2],
      vec[0]*mat[2][0] + vec[1]*mat[2][1] + vec[2]*mat[2][2]
    ]
    return result;
  }
`;

/*
 * Functions used by skeleton simulation for 3d transformations
 */
var skeleton_transform_functions = `
function createQuaternionFromYawPitchRoll(yaw, pitch, roll) {
  var sr, cr, sp, cp, sy, cy;

  var halfRoll = roll * 0.5;
  sr = Math.sin(halfRoll);
  cr = Math.cos(halfRoll);

  var halfPitch = pitch * 0.5;
  sp = Math.sin(halfPitch);
  cp = Math.cos(halfPitch);

  var halfYaw = yaw * 0.5;
  sy = Math.sin(halfYaw);
  cy = Math.cos(halfYaw);

  var result = {};

  result.x = cy * sp * cr + sy * cp * sr;
  result.y = sy * cp * cr - cy * sp * sr;
  result.z = cy * cp * sr - sy * sp * cr;
  result.w = cy * cp * cr + sy * sp * sr;

  return result;
}

// Creates a quaternion describing the rotation around
// the given axis
function quaternionFromAxisAngle(axis, angleDegrees) {
  const angleRadians = angleDegrees * Math.PI / 180;
  const halfAngle = angleRadians / 2;
  const sinHalfAngle = Math.sin(halfAngle);
  const cosHalfAngle = Math.cos(halfAngle);
  
  return {
      x: axis.x * sinHalfAngle,
      y: axis.y * sinHalfAngle,
      z: axis.z * sinHalfAngle,
      w: cosHalfAngle
  };
}

// Creates a quaternion describing the rotation around
// the up axis (axis Y)
function createQuaternionFromAngleUpAxis(angle) {
  var halfAngle = angle * 0.5;
  var cosHalfAngle = Math.cos(halfAngle);
  var sinHalfAngle = Math.sin(halfAngle);

  return {
    x: 0,
    y: sinHalfAngle,
    z: 0,
    w: cosHalfAngle,
  };
}

// Creates a 3x3 rotation matrix from a given quaternion
function createRotationMatrixFromQuaternion(quaternion) {
  const { x, y, z, w } = quaternion;
  const xx = x * x;
  const xy = x * y;
  const xz = x * z;
  const xw = x * w;
  const yy = y * y;
  const yz = y * z;
  const yw = y * w;
  const zz = z * z;
  const zw = z * w;

  return [
    1 - 2 * (yy + zz),
    2 * (xy - zw),
    2 * (xz + yw),
    2 * (xy + zw),
    1 - 2 * (xx + zz),
    2 * (yz - xw),
    2 * (xz - yw),
    2 * (yz + xw),
    1 - 2 * (xx + yy),
  ];
}

// Rotates a vector by the given 3x3 rotation matrix
function rotateVector(vector, rotationMatrix) {
  const x =
    vector.x * rotationMatrix[0] +
    vector.y * rotationMatrix[1] +
    vector.z * rotationMatrix[2];
  const y =
    vector.x * rotationMatrix[3] +
    vector.y * rotationMatrix[4] +
    vector.z * rotationMatrix[5];
  const z =
    vector.x * rotationMatrix[6] +
    vector.y * rotationMatrix[7] +
    vector.z * rotationMatrix[8];
  return { x: x, y: y, z: z };
}

function addVectors(v1, v2) {
  return { x: v1.x + v2.x, y: v1.y + v2.y, z: v1.z + v2.z };
}

function subtractVectors(v1, v2) {
  return { x: v1.x - v2.x, y: v1.y - v2.y, z: v1.z - v2.z };
}

function multiplyVector(vector, multiplier) {
  return {
    x: vector.x * multiplier,
    y: vector.y * multiplier,
    z: vector.z * multiplier,
  };
}

function quaternionMultiplication(q1, q2) {
  const w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
  const x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
  const y = q1.w * q2.y + q1.y * q2.w + q1.z * q2.x - q1.x * q2.z;
  const z = q1.w * q2.z + q1.z * q2.w + q1.x * q2.y - q1.y * q2.x;

  return { w: w, x: x, y: y, z: z };
}

function inverseQuaternion(q) {
  const magnitudeSquared = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  const inverseMagnitudeSquared = 1 / magnitudeSquared;
  return {
    x: -q.x * inverseMagnitudeSquared,
    y: -q.y * inverseMagnitudeSquared,
    z: -q.z * inverseMagnitudeSquared,
    w: q.w * inverseMagnitudeSquared,
  };
}

// Creates a 3x3 rotation matrix based on the angle that describes
// rotation around the up axis (axis Y)
function createRotationMatrixFromAngleUpAxis(angle) {
  var cosAngle = Math.cos(angle);
  var sinAngle = Math.sin(angle);
  return [cosAngle, 0, sinAngle, 0, 1, 0, -sinAngle, 0, cosAngle];
}

function eulerVectorToRadians(eulerVector) {
  return multiplyVector(eulerVector, Math.PI / 180);
}

// Converts a quaternion to an euler angles representation
function toEulerAngles(q) {
  let yaw, pitch, roll;
  let test = q.x * q.y + q.z * q.w;
  const singularity = 0.499;
  if (test > singularity || test < -singularity) {
    yaw = (test < -singularity ? -2 : 2) * Math.atan2(q.x, q.w);
    pitch = ((test < -singularity ? -1 : 1) * Math.PI) / 2;
    roll = 0;
  } else {
    let sqx = q.x * q.x;
    let sqy = q.y * q.y;
    let sqz = q.z * q.z;
    yaw = Math.atan2(2 * q.y * q.w - 2 * q.x * q.z, 1 - 2 * sqy - 2 * sqz);
    pitch = Math.asin(2 * test);
    roll = Math.atan2(2 * q.x * q.w - 2 * q.y * q.z, 1 - 2 * sqx - 2 * sqz);
  }
  return { x: roll, y: yaw, z: pitch };
}`;

/*
 * Functions used by skeleton simulation to control the simulation
 */
var skeleton_helper_functions = `
// Smoothing sigmoid function for bone displacement
function sigmoid(value) {
  const k = Math.exp(value);
  return (k / (1.0 + k) - 0.5) * 2;
}

// https://en.wikipedia.org/wiki/Smoothstep
function smootherstep(time) {
  return time * time * time * (time * (time * 6 - 15) + 10);
}

// Computes and adds together all sines in a simulation factor (+ constant)
function computeSimulationFactor(factor, time) {
  let sinesSum = 0;
  if (factor.sines != null && factor.sines.length > 0) {
    sinesSum = factor.sines.reduce(
      (accumulator, sine) =>
        accumulator + sine.a * Math.sin(sine.f * time + sine.p),
      0
    );
  }
  return factor.constant + sinesSum;
}

// Interpolates between two simulation factors based on given transition portion
function interpolateFactor(time, factor1, factor2, transitionPortion) {
  const value1 = computeSimulationFactor(factor1, time);
  if (transitionPortion <= 0) {
    return value1;
  }
  const value2 = computeSimulationFactor(factor2, time);
  return value1 * (1 - transitionPortion) + value2 * transitionPortion;
}`;

// Class representing a bone, it is expected to be attached to another bone or legs
class SkeletonBone {
  constructor(length, attachedOffset, attachedRotationEuler, maxAngles) {
    // Length of the bone
    this.length = length;
    // How far from the end of the parent part it is attached (0 = end, 1 = start)
    this.attachedOffset = attachedOffset;
    // What is the relative rotation from the parent part
    this.attachedRotation = createQuaternionFromYawPitchRoll(
      attachedRotationEuler.y,
      attachedRotationEuler.x,
      attachedRotationEuler.z
    );
    // Maximum angle displacements for this bone
    this.maxAngles = maxAngles;
  }

  // Calculates the base position and rotation of this bone, based on the previously calculated bone and the given roll, angle and amount values.
  move(previous, angle, amount, roll) {
    let location = {
      x: previous.location.x,
      y: previous.location.y,
      z: previous.location.z,
    };
    // Calculate the starting position of this bone
    const rotationMatrix = createRotationMatrixFromQuaternion(
      previous.rotation
    );
    const rotatedParentOffset = rotateVector(
      { x: 0, y: previous.length * (1 - this.attachedOffset), z: 0 },
      rotationMatrix
    );
    location = addVectors(location, rotatedParentOffset);

    // Calculate the rotation of this bone
    const yRotation = sigmoid(roll) * this.maxAngles.y;
    const xRotation = Math.cos(angle) * sigmoid(amount) * this.maxAngles.x;
    const zRotation = Math.sin(angle) * sigmoid(amount) * this.maxAngles.z;
    const yawPitchRotation = createQuaternionFromYawPitchRoll(
      0,
      xRotation,
      zRotation
    );
    const rollRotation = createQuaternionFromYawPitchRoll(yRotation, 0, 0);
    let rotation = quaternionMultiplication(yawPitchRotation, rollRotation);
    rotation = quaternionMultiplication(this.attachedRotation, rotation);
    rotation = quaternionMultiplication(previous.rotation, rotation);

    return { location: location, rotation: rotation, length: this.length };
  }
}

// Class representing the legs of a skeleton
class SkeletonLegs {
  constructor(length) {
    // Length of the legs
    this.length = length;
    // Last location of the legs
    this.location = { x: 0, y: 0, z: 0 };
  }

  // Moves and rotates the legs
  move(deltaTime, velocity, direction) {
    const rotationMatrix = createRotationMatrixFromAngleUpAxis(direction);
    const rotatedVelocity = rotateVector(velocity, rotationMatrix);
    this.location = addVectors(
      this.location,
      multiplyVector(rotatedVelocity, deltaTime)
    );
    return {
      location: this.location,
      rotation: createQuaternionFromAngleUpAxis(direction),
      length: this.length,
    };
  }
}

// Class representing the simulated device in the skeleton
class SkeletonPhone {
  constructor(attachedOffset, attachedRotationEuler) {
    this.attachedOffset = attachedOffset;
    this.attachedRotation = createQuaternionFromYawPitchRoll(
      attachedRotationEuler.y,
      attachedRotationEuler.x,
      attachedRotationEuler.z
    );
  }

  // Calculates the position and rotation of the device
  move(previous) {
    let location = {
      x: previous.location.x,
      y: previous.location.y,
      z: previous.location.z,
    };
    const rotationMatrix = createRotationMatrixFromQuaternion(
      previous.rotation
    );
    const rotatedParentOffset = rotateVector(
      { x: 0, y: previous.length * (1 - this.attachedOffset), z: 0 },
      rotationMatrix
    );
    location = addVectors(location, rotatedParentOffset);
    let rotation = quaternionMultiplication(
      previous.rotation,
      this.attachedRotation
    );
    return { location: location, rotation: rotation };
  }
}

// Represents a simulated skeleton
class Skeleton {
  constructor() {
    this.legs = new SkeletonLegs(1);
    this.bones = [
      // Torso
      new SkeletonBone(
        0.55,
        0,
        eulerVectorToRadians({ x: 0, y: 0, z: 0 }),
        eulerVectorToRadians({ x: 15, y: 25, z: 28 })
      ),
      // Shoulder
      new SkeletonBone(
        0.2,
        0.1,
        eulerVectorToRadians({ x: 98, y: 15, z: 0 }),
        eulerVectorToRadians({ x: 17, y: 0, z: 0 })
      ),
      // Upper Arm
      new SkeletonBone(
        0.3,
        0,
        eulerVectorToRadians({ x: 15, y: 10, z: -35 }),
        eulerVectorToRadians({ x: 65, y: 60, z: 70 })
      ),
      // Lower Arm
      new SkeletonBone(
        0.27,
        0,
        eulerVectorToRadians({ x: 0, y: 0, z: -70 }),
        eulerVectorToRadians({ x: 0, y: 0, z: 70 })
      ),
      // Hand
      new SkeletonBone(
        0.21,
        0,
        eulerVectorToRadians({ x: 0, y: 0, z: -15 }),
        eulerVectorToRadians({ x: 25, y: 90, z: 75 })
      ),
    ];
    this.phone = new SkeletonPhone(
      0.5,
      eulerVectorToRadians({ x: 0, y: 90, z: -90 })
    );

    // Last 3 results are stored for calculating accelerations and gyroscope values
    this.currentResults = null;
    this.previousResults = null;
    this.beforePreviousResults = null;
  }

  update(time, parameters1, parameters2, transitionPortion) {
    // Change the scale of time to seconds instead of milliseconds
    time /= 1000;
    // Shift the three stored results down
    this.beforePreviousResults = this.previousResults;
    this.previousResults = this.currentResults;
    // Get delta time for leg movement, default to 0
    let deltaTime = 0;
    if (this.previousResults != null) {
      deltaTime = time - this.previousResults.time;
    }
    // Make transitions between parameters smoother
    transitionPortion = smootherstep(transitionPortion);
    // Simulate the legs
    const legsVelocity = {
      x: interpolateFactor(
        time,
        parameters1.legs.velocityX,
        parameters2.legs.velocityX,
        transitionPortion
      ),
      y: interpolateFactor(
        time,
        parameters1.legs.velocityY,
        parameters2.legs.velocityY,
        transitionPortion
      ),
      z: interpolateFactor(
        time,
        parameters1.legs.velocityZ,
        parameters2.legs.velocityZ,
        transitionPortion
      ),
    };
    const legsDirection = interpolateFactor(
      time,
      parameters1.legs.direction,
      parameters2.legs.direction,
      transitionPortion
    );
    let lastBoneResults = this.legs.move(
      deltaTime,
      legsVelocity,
      legsDirection
    );
    // Simulate all of the bones
    for (let i = 0; i < this.bones.length; i++) {
      const angle = interpolateFactor(
        time,
        parameters1.bones[i].angle,
        parameters2.bones[i].angle,
        transitionPortion
      );
      const amount = interpolateFactor(
        time,
        parameters1.bones[i].amount,
        parameters2.bones[i].amount,
        transitionPortion
      );
      const roll = interpolateFactor(
        time,
        parameters1.bones[i].roll,
        parameters2.bones[i].roll,
        transitionPortion
      );
      lastBoneResults = this.bones[i].move(
        lastBoneResults,
        angle,
        amount,
        roll
      );
    }
    // Calculate the final results
    lastBoneResults = this.phone.move(lastBoneResults);
    this.currentResults = {
      location: lastBoneResults.location,
      rotation: lastBoneResults.rotation,
      time: time,
    };
    return this.currentResults;
  }
}

// Base class for sensor generators, handles updating the skeleton and switching parameters
class SensorGenerator {
  constructor() {
    this.parameters = JSON.parse(skeleton_parameters_string);
    this.transitionStart = 0;
    this.transitionEnd = 0;
    this.skeleton = new Skeleton();
    this.random = new SenPseudoRandom();
    this.parametersStart = null;
    this.parametersEnd = null;
  }

  // Updates the skeleton based on current transition variables and passed time
  updateSkeleton(time) {
    const transitionPortion = Math.max(
      0,
      Math.min(
        1,
        (time - this.transitionStart) /
          (this.transitionEnd - this.transitionStart)
      )
    );
    return this.skeleton.update(
      time,
      this.parametersStart,
      this.parametersEnd,
      transitionPortion
    );
  }

  // If needed, updates (or intializes) the transition variables
  updateTransition(time) {
    if (this.transitionEnd > time) {
      return;
    }
    if (this.parametersEnd == null) {
      this.parametersEnd =
        this.parameters[
          Math.min(
            this.parameters.length - 1,
            Math.floor(this.parameters.length * this.random.next())
          )
        ];
    }
    let transition = this.getNextTransition(time, this.transitionEnd);
    this.transitionEnd = transition.end;
    this.transitionStart = transition.start;
    this.parametersStart = this.parametersEnd;
    this.parametersEnd =
      this.parameters[
        Math.min(
          this.parameters.length - 1,
          Math.floor(this.parameters.length * transition.params)
        )
      ];
  }

  // Gets the start and end time of the next transition, as well as the 0-1 range for selecting next parameters.
  getNextTransition(time) {
    let params, lastParams, transitionStartOffset;
    let transitionEnd = this.transitionEnd;
    do {
      transitionEnd += this.random.nextMinMax(15000, 45000);
      transitionStartOffset = this.random.nextMinMax(2500, 6000);
      lastParams = params;
      params = this.random.next();
    } while (transitionEnd <= time);
    return {
      start: transitionEnd - transitionStartOffset,
      end: transitionEnd,
      params: params,
      lastParams: lastParams,
    };
  }
}
