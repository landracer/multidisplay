/*
    Copyright 2009-14 Stephan Martin, Dominik Gummel

    This file is part of Multidisplay.

    Multidisplay is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Multidisplay is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Multidisplay.  If not, see <http://www.gnu.org/licenses/>.

    MultiDisplayDefinesVR6.h
    Created on: 31.07.2014
*/

/**
 * @file MultidisplayDefinesVR6.h
 * @brief ECU-specific configuration for VW VR6 with Bosch Motronic ECU.
 *
 * This header is included by MultidisplayDefines.h when VR6_MOTRONIC is
 * defined.  It sets sensor calibrations, K-line options, boost/lambda
 * sensor types, RPM/speed factors, and the number of attached EGT probes
 * specific to the VR6 engine installation.
 *
 * Select exactly ONE sub-variant:
 *   VR6_MOTRONIC_M2X  – Motronic M2.x (older, 8-bit throttle)
 *   VR6_MOTRONIC_M381 – Motronic M3.8.1 (16-bit throttle, more common)
 */

#ifndef MULTIDISPLAYDEFINESVR6_H_
#define MULTIDISPLAYDEFINESVR6_H_

/* --- Motronic sub-variant (choose one) --- */
/* #define VR6_MOTRONIC_M2X */
#define VR6_MOTRONIC_M381

/* ========================================================================== */
/*  K-Line Diagnostics (EXPERIMENTAL — disabled by default)                   */
/* ========================================================================== */
/*
 * KWP1281_KLINE       – enable KWP1281 diagnostics over the K-line.
 * KWP1281_KLINE_DEBUG – print raw K-line traffic to Serial for debugging.
 * KWP1281_KLINE_MIRROR_TO_SERIAL – echo K-line data to USB serial.
 *
 * WARNING: K-line support is experimental and may be unstable on some ECUs.
 */
/* #define KWP1281_KLINE */
/* #define KWP1281_KLINE_DEBUG 1 */
/* #define KWP1281_KLINE_MIRROR_TO_SERIAL 1 */

/* ========================================================================== */
/*  Boost MAP Sensor Selection (choose one)                                   */
/* ========================================================================== */
/*
 * The boost pressure is read from an external MCP3208 12-bit ADC channel.
 * Select the sensor type installed in your intake plumbing:
 *
 *   USE_DIGIFANT_MAPSENSOR    – use Digifant's internal MAP sensor
 *   BOOST_PLX_SMVACBOOST      – PLX SM-VAC/BOOST wideband sensor
 *   BOOST_MOTOROLA_MPX4250    – Motorola MPX4250 (250 kPa absolute)
 *   BOOST_FREESCALE_MPXA6400A – Freescale MPXA6400A (400 kPa absolute)
 *   BOOST_BOSCH_200KPA        – Bosch 200 kPa sensor (on BOOST2PIN)
 */
/* #define USE_DIGIFANT_MAPSENSOR */
/* #define BOOST_PLX_SMVACBOOST */
/* #define BOOST_MOTOROLA_MPX4250 */
#define BOOST_FREESCALE_MPXA6400A
/* #define BOOST_BOSCH_200KPA */          /* connected to BOOST2PIN */

/* ========================================================================== */
/*  Lambda (O₂) Sensor Selection                                             */
/* ========================================================================== */
/*
 * LAMBDA_WIDEBAND     – set if using a wideband (AFR) sensor.
 *                        Then choose the specific wideband model:
 *   SLCOEM             – 14point7.com SLC-OEM (AFR = 2×V + 10)
 *   LAMBDA_PLX_SMAFR   – PLX SM-AFR (AFR = 2×V + 10)
 *   LAMBDA_INNOVATE_LC1 – Innovate LC-1 (AFR = 3×V + 7.35)
 *   LAMBDASTANDALONE   – lambda wired directly to an Arduino analog pin
 *                         (10-bit ADC, not MCP3208)
 *
 * If LAMBDA_WIDEBAND is NOT defined, a narrow-band lambda sensor is assumed.
 */
#define LAMBDA_WIDEBAND
#define SLCOEM
/* #define LAMBDA_PLX_SMAFR */
/* #define LAMBDA_INNOVATE_LC1 */
/* #define LAMBDASTANDALONE A2 */

/* ========================================================================== */
/*  RPM Signal Configuration                                                  */
/* ========================================================================== */
/*
 * Choose ONE RPM source:
 *   USE_RPM_ONBOARD    – RPM read from the onboard signal-conditioning
 *                         circuit (analog input through MCP3208).
 *   USE_DIGIFANT_RPM   – RPM extracted from the Digifant K-line stream.
 *
 * RPMFACTOR – calibration multiplier from ADC reading to actual RPM.
 *   Depends on the signal-conditioning resistor network:
 *     20 kΩ pull-up: factor ≈ 5.21 (4-cyl) or 9.2 (6-cyl VR6)
 *     75 kΩ pull-up: factor ≈ 2.34 (4-cyl Digifant)
 */
#define USE_RPM_ONBOARD
/* #define USE_DIGIFANT_RPM */
#define RPMFACTOR 9.2
/* #define RPMFACTOR 2.34 */

/* ========================================================================== */
/*  Vehicle Speed Configuration                                               */
/* ========================================================================== */
/*
 * SPEEDFACTOR           – multiplier from ADC reading to km/h.
 * SPEEDCORRECTIONFACTOR – post-calibration fudge factor for tire wear /
 *                          speedo drive ratio differences.
 */
#define SPEEDFACTOR 0.38
#define SPEEDCORRECTIONFACTOR 0.9
/* #define SPEEDFACTOR 0.390625 */
/* #define SPEEDCORRECTIONFACTOR 1 */

/* ========================================================================== */
/*  Type-K EGT Probes                                                         */
/* ========================================================================== */
/*
 * NUMBER_OF_ATTACHED_TYPE_K – how many Type-K thermocouples are wired
 *   to the analog multiplexer.  Maximum is MAX_ATTACHED_TYPE_K (8).
 *   VR6 with 6 cylinders typically has 6 probes (one per exhaust runner).
 */
#define NUMBER_OF_ATTACHED_TYPE_K 6

#endif /* MULTIDISPLAYDEFINESVR6_H_ */
