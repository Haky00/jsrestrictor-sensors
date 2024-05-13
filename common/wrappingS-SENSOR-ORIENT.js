/** \file
 * \brief Wrappers for the AbsoluteOrientationSensor and RelativeOrientationSensor
 *
 * \see https://www.w3.org/TR/orientation-sensor
 * \see https://www.w3.org/TR/orientation-sensor/#absoluteorientationsensor-model
 * \see https://www.w3.org/TR/orientation-sensor/#relativeorientationsensor-model
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
 * MOTIVATION
 * Device orientation sensors can be easily used for fingerprinting. As it highly
 * unlikely that two devices visiting the same site will be oriented exactly
 * the same, the orientation itself can serve as a fingerprint.
 *
 *
 * WRAPPING
 * AbsoluteOrientationSensor returns a quaterion decribing the physical
 * orientation of the device in relation to the Earth's reference coordinate
 * system. The faked orientation of the device is obtained by simulating the
 * movement of a human skeleton.
 *
 * RelativeOrientationSensor also describes the orientation, but without
 * regard to the Earth's reference coordinate system. We suppose the coordinate
 * system is chosen at the beginning of the sensor instance creation.
 * As we observed, no matter how the device is oriented, there is always a slight
 * difference from the AbsoluteOrientationSensor's in at least one axis.
 * When the device moves, both sensors' readings change. But their difference
 * should be always constant. And thus, we pseudorandomly generate a deviation
 * from the Earth's reference coordinate system. And for each reading, we
 * take the values from the fake AbsoluteOrientationSensor and modify them
 * by the constant deviation.
 *
 * POSSIBLE IMPROVEMENTS
 * Study the supported coordinate systems of the RelativeOrientationSensor
 * and modify the wrapper behavior if needed.
 */

/*
 * Create private namespace
 */
(function () {
  /*
   * \brief Initialization of data for storing sensor readings
   */
  var init_data = `
    var currentReading = currentReading || {quaternion: null, fake_quaternion: null, fake_quaternion_rel: null, timestamp: null};
    var previousReading = previousReading || {quaternion: null, fake_quaternion: null, fake_quaternion_rel: null, timestamp: null};
    var debugMode = false;

    const TWOPI = 2 * Math.PI;
    `;

  /*
   * \brief Property getters of the original sensor object
   */
  var orig_getters = `
    var origGetQuaternion = Object.getOwnPropertyDescriptor(OrientationSensor.prototype, "quaternion").get;
    var origGetTimestamp = Object.getOwnPropertyDescriptor(Sensor.prototype, "timestamp").get;
    `;

  /*
   * \brief Fake device orientation generator class
   *        (based on skeleton movement simulation)
   */
  class OrientationGenerator extends SensorGenerator {
    constructor() {
      super();
      this.DEVIATION_MIN = 0;
      this.DEVIATION_MAX = (Math.PI / 2 / 90) * 10; // 10°

      this.relativeOrientation = { x: 0, y: 0, z: 0 };
      this.absoluteOrientation = { x: 0, y: 0, z: 0 };
      this.deviation = createQuaternionFromYawPitchRoll(
        this.generateDeviation(),
        this.generateDeviation(),
        this.generateDeviation()
      );
      // Skeleton simulation is rotated by -90° X axis
      this.additionalRotation = quaternionFromAxisAngle(
        { x: 1, y: 0, z: 0 },
        90
      );
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
      // Skeleton simulation is rotated by -90° X axis
      this.absoluteOrientation = quaternionMultiplication(
        this.additionalRotation,
        result.rotation
      );
      // Add deviation to relative orientation sensor
      this.relativeOrientation = quaternionMultiplication(
        this.absoluteOrientation,
        this.deviation
      );
    }

    /*
     * \brief Generates the rotation deviation
     */
    generateDeviation() {
      var devi =
        sen_prng() * (this.DEVIATION_MAX - this.DEVIATION_MIN) +
        this.DEVIATION_MIN;
      devi *= Math.round(sen_prng()) ? 1 : -1;
      return devi;
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
    // from the previous sample. If so, we need to update the faked quaternion
    let previousTimestamp = previousReading.timestamp;
    let currentTimestamp = origGetTimestamp.call(sensorObject);
    if (debugMode) {
      // [!] Debug mode: overriding timestamp
      // This allows test suites to set a custom timestamp externally
      // by modifying the property of the Magnetometer object directly.
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
    //       in improvements of the magnetic field generator
    currentReading.orig_quaterion = origGetQuaternion.call(sensorObject);
    currentReading.timestamp = currentTimestamp;
    orientationGenerator.updateValues(currentTimestamp);
    currentReading.fake_quaternion = [
      orientationGenerator.absoluteOrientation.x,
      orientationGenerator.absoluteOrientation.y,
      orientationGenerator.absoluteOrientation.z,
      orientationGenerator.absoluteOrientation.w,
    ];
    currentReading.fake_quaternion_rel = [
      orientationGenerator.relativeOrientation.x,
      orientationGenerator.relativeOrientation.y,
      orientationGenerator.relativeOrientation.z,
      orientationGenerator.relativeOrientation.w,
    ];

    if (debugMode) {
      console.debug(quaternionGenerator);
    }
  }

  /*
   * \brief Initializes the related generators
   */
  var generators = `
    // Initialize the quaternion generator, if not initialized before
    var orientationGenerator = orientationGenerator || new OrientationGenerator();
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
    OrientationGenerator +
    updateReadings;
  var hc = init_data + orig_getters + helping_functions + generators;

  var wrappers = [
    {
      parent_object: "OrientationSensor.prototype",
      parent_object_property: "quaternion",
      wrapped_objects: [],
      helping_code: hc,
      post_wrapping_code: [
        {
          code_type: "object_properties",
          parent_object: "OrientationSensor.prototype",
          parent_object_property: "quaternion",
          wrapped_objects: [],
          /**  \brief replaces OrientationSensor.quaternion getter to return a faked value
           */
          wrapped_properties: [
            {
              property_name: "get",
              property_value: `
          function() {
            updateReadings(this);
            if (this.__proto__.constructor.name === 'AbsoluteOrientationSensor') {
              // AbsoluteOrientationSensor
              return currentReading.fake_quaternion;
            } else {
              // RelativeOrientationSensor
              return currentReading.fake_quaternion_rel;
            }
          }`,
            },
          ],
        },
      ],
    },
  ];
  add_wrappers(wrappers);
})();
