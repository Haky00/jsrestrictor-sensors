/** \file
 * \brief Wrappers for the Accelerometer Sensor, LinearAccelerationSensor,
 * and GravitySensor
 *
 * \see https://www.w3.org/TR/accelerometer/
 * \see https://www.w3.org/TR/accelerometer/#linearaccelerationsensor
 * \see https://www.w3.org/TR/accelerometer/#gravitysensor
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
 * MOTIVATION
 * Readings from the Accelerometer, LinearAccelerationSensor, and GravitySensor
 * of the Generic Sensor API should be secured as they provide a potentially
 * valuable data for creating fingerprints. There are multiple options.
 * A unique fingerprint can be obtained by describing the device's vibrations
 * (See https://link.springer.com/chapter/10.1007/978-3-319-30806-7_7).
 * Using trajectory inference and matching of the model to map data, one may
 * use the readings from the Accelerometer to determing the device's position
 * (See https://www.researchgate.net/publication/220990763_ACComplice_Location_
 * inference_using_accelerometers_on_smartphones).
 * Accelerometer readings can also be used for determining human walking patterns
 * (See https://www.researchgate.net/publication/322835708_Classifying_Human_
 *  Walking_Patterns_using_Accelerometer_Data_from_Smartphone).
 *
 *
 * WRAPPING
 * The wrapper replaces the "XYZ" getters of the Accelerometer sensor,
 * LinearAccelerationSensor, and GravitySensor. Movement of the device is
 * obtained by simulating a human skeleton. Last 3 simulated positions are 
 * used for calculating the linear acceleration. The simulated sensor computes 
 * both the linear and gravity parts of acceleration.
 */

/*
 * Create private namespace
 */
(function () {
  /*
   * \brief Initialization of data for storing sensor readings
   */
  var init_data = `
    var currentReading = currentReading || {orig_x: null, orig_y: null, orig_z: null, timestamp: null,
                      fake_x: null, fake_y: null, fake_z: null, gVector: null};
    var previousReading = previousReading || {orig_x: null, orig_y: null, orig_z: null, timestamp: null,
                      fake_x: null, fake_y: null, fake_z: null, gVector: null};
    var emulateStationaryDevice = (typeof args === 'undefined') ? true : args[0];
    var debugMode = false;

    const TWOPI = 2 * Math.PI;
    `;

  /*
   * \brief Property getters of the original sensor object
   */
  var orig_getters = `
    var origGetX = Object.getOwnPropertyDescriptor(Accelerometer.prototype, "x").get;
    var origGetY = Object.getOwnPropertyDescriptor(Accelerometer.prototype, "y").get;
    var origGetZ = Object.getOwnPropertyDescriptor(Accelerometer.prototype, "z").get;
    var origGetTimestamp = Object.getOwnPropertyDescriptor(Sensor.prototype, "timestamp").get;
    `;

  /*
   * \brief Fake device acceleration generator class
   *        (based on skeleton movement simulation)
   */
  class AccelerationGenerator extends SensorGenerator {
    constructor() {
      super();
      this.acceleration = { x: 0, y: 0, z: 0 };
      this.gravity = { x: 0, y: 0, z: 0 };
      this.referenceGravity = { x: 0, y: -9.8, z: 0 };
      this.lastTime = -1;
    }

    updateValues(time) {
      // Skip updating values if requested time is not later
      if (this.lastTime >= time) {
        return;
      }
      this.lastTime = time;
      // Perform skeleton updates
      this.updateTransition(time);
      const result = this.updateSkeleton(time);
      // We need at least 3 results to calculate acceleration
      if (
        this.skeleton.previousResults == null ||
        this.skeleton.beforePreviousResults == null
      ) {
        this.acceleration = { x: 0, y: 0, z: 0 };
        return;
      }
      // Timestamps of last 3 results
      const t1 = this.skeleton.beforePreviousResults.time;
      const t2 = this.skeleton.previousResults.time;
      const t3 = result.time;
      // Positions of last 3 results
      const p1 = this.skeleton.beforePreviousResults.location;
      const p2 = this.skeleton.previousResults.location;
      const p3 = result.location;
      // Displacements
      const d1 = subtractVectors(p2, p1);
      const d2 = subtractVectors(p3, p2);
      // Rotational matrix from newest rotation
      const rotationMatrix = createRotationMatrixFromQuaternion(
        inverseQuaternion(result.rotation)
      );
      // Relative velocities scaled by time
      const v1 = multiplyVector(
        rotateVector(d1, rotationMatrix),
        1 / (t2 - t1)
      );
      const v2 = multiplyVector(
        rotateVector(d2, rotationMatrix),
        1 / (t3 - t2)
      );
      // Acceleration from difference in velocities
      this.acceleration = multiplyVector(
        subtractVectors(v2, v1),
        2 / (t3 - t1)
      );
      // Rotate gravity vector based on device rotation
      this.gravity = rotateVector(this.referenceGravity, rotationMatrix);
    }
  }

  /*
   * \brief Updates the stored (both real and fake) sensor readings
   *        according to the data from the sensor object.
   *
   * \param The sensor object
   */
  function updateReadings(sensorObject) {
    // We need the original reading's timestamp to see if it differs
    // from the previous sample. If so, we need to update the faked x,y,z
    let currentTimestamp = origGetTimestamp.call(sensorObject);
    let previousTimestamp = previousReading.timestamp;

    if (debugMode) {
      // [!] Debug mode: overriding timestamp
      // This allows test suites to set a custom timestamp externally
      // by modifying the property of the sensor object directly.
      currentTimestamp = sensorObject.timestamp;
    }

    if (currentTimestamp === previousTimestamp) {
      // No new reading, nothing to update
      return;
    }

    // Rotate the readings: previous <- current
    previousReading = JSON.parse(JSON.stringify(currentReading));

    // Update current reading
    // NOTE: Original values are also stored for possible future use
    currentReading.orig_x = origGetX.call(sensorObject);
    currentReading.orig_y = origGetY.call(sensorObject);
    currentReading.orig_z = origGetZ.call(sensorObject);
    currentReading.timestamp = currentTimestamp;

    accelerationGenerator.updateValues(currentTimestamp);

    currentReading.fake_x = accelerationGenerator.acceleration.x;
    currentReading.fake_y = accelerationGenerator.acceleration.y;
    currentReading.fake_z = accelerationGenerator.acceleration.z;
    currentReading.fake_gVector = [
      accelerationGenerator.gravity.x,
      accelerationGenerator.gravity.y,
      accelerationGenerator.gravity.z,
    ];

    if (debugMode) {
      console.debug(accelerationGenerator);
    }
  }

  /*
   * \brief Initializes the related generators
   */
  var generators = `
    // Initialize the data generator, if not initialized before
    var accelerationGenerator = accelerationGenerator || new AccelerationGenerator();
    `;

  var helping_functions =
    sensorapi_prng_functions +
    device_orientation_functions +
    skeleton_parameters +
    skeleton_transform_functions +
    skeleton_helper_functions +
    Skeleton +
    SkeletonBone +
    SkeletonPhone +
    SkeletonLegs +
    SenPseudoRandom +
    SensorGenerator +
    AccelerationGenerator +
    updateReadings;
  var hc = init_data + orig_getters + helping_functions + generators;

  var wrappers = [
    {
      parent_object: "Accelerometer.prototype",
      parent_object_property: "x",
      wrapped_objects: [],
      helping_code: hc,
      post_wrapping_code: [
        {
          code_type: "object_properties",
          parent_object: "Accelerometer.prototype",
          parent_object_property: "x",
          wrapped_objects: [],
          /**  \brief replaces Sensor.prototype.x getter to return a faked value
           */
          wrapped_properties: [
            {
              property_name: "get",
              property_value: `
              function() {
                updateReadings(this);
                if (this.__proto__.constructor.name === 'GravitySensor') {
                  return fixedNumber(currentReading.fake_gVector[0], 1);
                } else if (this.__proto__.constructor.name === 'LinearAccelerationSensor') {
                  return fixedNumber(currentReading.fake_x, 1);
                }
                return fixedNumber(currentReading.fake_x + currentReading.fake_gVector[0], 1);
              }`,
            },
          ],
        },
      ],
    },
    {
      parent_object: "Accelerometer.prototype",
      parent_object_property: "y",
      wrapped_objects: [],
      helping_code: hc,
      post_wrapping_code: [
        {
          code_type: "object_properties",
          parent_object: "Accelerometer.prototype",
          parent_object_property: "y",
          wrapped_objects: [],
          /**  \brief replaces Sensor.prototype.y getter to return a faked value
           */
          wrapped_properties: [
            {
              property_name: "get",
              property_value: `
              function() {
                updateReadings(this);
                if (this.__proto__.constructor.name === 'GravitySensor') {
                  return fixedNumber(currentReading.fake_gVector[1], 1);
                } else if (this.__proto__.constructor.name === 'LinearAccelerationSensor') {
                  return fixedNumber(currentReading.fake_y, 1);
                }
                return fixedNumber(currentReading.fake_y + currentReading.fake_gVector[1], 1);
              }`,
            },
          ],
        },
      ],
    },
    {
      parent_object: "Accelerometer.prototype",
      parent_object_property: "z",
      wrapped_objects: [],
      helping_code: hc,
      post_wrapping_code: [
        {
          code_type: "object_properties",
          parent_object: "Accelerometer.prototype",
          parent_object_property: "z",
          wrapped_objects: [],
          /**  \brief replaces Sensor.prototype.z getter to return a faked value
           */
          wrapped_properties: [
            {
              property_name: "get",
              property_value: `
              function() {
                updateReadings(this);
                if (this.__proto__.constructor.name === 'GravitySensor') {
                  return fixedNumber(currentReading.fake_gVector[2], 1);
                } else if (this.__proto__.constructor.name === 'LinearAccelerationSensor') {
                  return fixedNumber(currentReading.fake_z, 1);
                }
                return fixedNumber(currentReading.fake_z + currentReading.fake_gVector[2], 1);
              }`,
            },
          ],
        },
      ],
    },
  ];
  add_wrappers(wrappers);
})();
