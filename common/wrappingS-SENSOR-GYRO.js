/** \file
 * \brief Wrappers for the Gyroscope Sensor
 *
 * \see https://www.w3.org/TR/Gyroscope/
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
 * Gyroscope readings can be used for speech recognition: https://crypto.stanford.edu/gyrophone/
 * and various fingerprinting operations. For stationary devices, the resonance of the unique internal or
 * external sounds affects angular velocities affect the Gyroscope and allow to create a fingerprint:
 * https://www.researchgate.net/publication/356678825_Mobile_Device_Fingerprint_Identification_Using_Gyroscope_Resonance
 * For moving devices, one of the options is using the Gyroscope analyze human walking patterns:
 * https://www.ncbi.nlm.nih.gov/pmc/articles/PMC7071017/
 *
 *
 * WRAPPING
 * The wrapper replaces the "XYZ" getters of the Gyroscope sensor. Rotations 
 * of the device are obtained by simulating a human skeleton. Last 2 simulated 
 * rotations are used for calculating the angular velocity.
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
                      fake_x: null, fake_y: null, fake_z: null};
    var previousReading = previousReading || {orig_x: null, orig_y: null, orig_z: null, timestamp: null,
                      fake_x: null, fake_y: null, fake_z: null};
    var emulateStationaryDevice = (typeof args === 'undefined') ? true : args[0];
    var debugMode = false;

    const TWOPI = 2 * Math.PI;
    `;

  /*
   * \brief Property getters of the original sensor object
   */
  var orig_getters = `
    var origGetX = Object.getOwnPropertyDescriptor(Gyroscope.prototype, "x").get;
    var origGetY = Object.getOwnPropertyDescriptor(Gyroscope.prototype, "y").get;
    var origGetZ = Object.getOwnPropertyDescriptor(Gyroscope.prototype, "z").get;
    var origGetTimestamp = Object.getOwnPropertyDescriptor(Sensor.prototype, "timestamp").get;
    `;

  /*
   * \brief Fake device gyroscope generator class
   *        (based on skeleton movement simulation)
   */
  class GyroscopeGenerator extends SensorGenerator {
    constructor() {
      super();
      this.lastTime = -1;
      this.gyroscope = { x: 0, y: 0, z: 0 };
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
      // We need at least 2 results to calculate angular velocity
      if (this.skeleton.previousResults == null) {
        this.gyroscope = { x: 0, y: 0, z: 0 };
        return;
      }
      // Last 2 rotations
      const r1 = this.skeleton.previousResults.rotation;
      const r2 = result.rotation;
      // Delta time between them
      const dt = result.time - this.skeleton.previousResults.time;
      // Get quaternion describing the difference
      const rotation = quaternionMultiplication(inverseQuaternion(r1), r2);
      // Convert difference to euler angles
      const axisAngle = toEulerAngles(rotation);
      // Scale it by delta time
      this.gyroscope = multiplyVector(axisAngle, 1 / dt);
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
    let previousTimestamp = previousReading.timestamp;
    let currentTimestamp = origGetTimestamp.call(sensorObject);

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

    gyroscopeGenerator.updateValues(currentTimestamp);

    currentReading.fake_x = gyroscopeGenerator.gyroscope.x;
    currentReading.fake_y = gyroscopeGenerator.gyroscope.y;
    currentReading.fake_z = gyroscopeGenerator.gyroscope.z;

    if (debugMode) {
      console.debug(gyroscopeGenerator);
    }
  }

  /*
   * \brief Initializes the related generators
   */
  var generators = `
    // Initialize the data generator, if not initialized before
    var gyroscopeGenerator = gyroscopeGenerator || new GyroscopeGenerator();
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
    GyroscopeGenerator +
    updateReadings;
  var hc = init_data + orig_getters + helping_functions + generators;

  var wrappers = [
    {
      parent_object: "Gyroscope.prototype",
      parent_object_property: "x",
      wrapped_objects: [],
      helping_code: hc,
      post_wrapping_code: [
        {
          code_type: "object_properties",
          parent_object: "Gyroscope.prototype",
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
                return currentReading.fake_x;
              }`,
            },
          ],
        },
      ],
    },
    {
      parent_object: "Gyroscope.prototype",
      parent_object_property: "y",
      wrapped_objects: [],
      helping_code: hc,
      post_wrapping_code: [
        {
          code_type: "object_properties",
          parent_object: "Gyroscope.prototype",
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
                return currentReading.fake_y;
              }`,
            },
          ],
        },
      ],
    },
    {
      parent_object: "Gyroscope.prototype",
      parent_object_property: "z",
      wrapped_objects: [],
      helping_code: hc,
      post_wrapping_code: [
        {
          code_type: "object_properties",
          parent_object: "Gyroscope.prototype",
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
                return currentReading.fake_z;
              }`,
            },
          ],
        },
      ],
    },
  ];
  add_wrappers(wrappers);
})();
