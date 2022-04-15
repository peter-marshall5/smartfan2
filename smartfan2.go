package main

import (
  "fmt"
  "os"
  "time"
  "math"
  "strconv"
  "strings"
  "os/signal"
  "syscall"
  "io/ioutil"
  "flag"
)

const (
  lowTemp = float64(55)
  medTemp = float64(62)
  highTemp = float64(70)
  throttleTemp = float64(91)
  dangerousTemp = float64(94)
  tempRiseThreshold = float64(2)
  tempDropThreshold = float64(12)
  minMode1Speed = float64(30)
)

var speedSatisfied = false
var speedTarget = float64(10)
var currSpeed = float64(0)
// 0 = passive, 1 = active
var mode = 1
var currTemp = float64(20)
var oldTemp = float64(0)
var lastECVal = 100
var errorAccumulation = float64(0)
var lastError = float64(0)

var gracefulQuitTried = false

var thermalZone string
var ecPath string
var ecAddr int64
var manualAddr int64
var readAddr int64
var ecMin float64
var ecMax float64
var readMin int64
var readMax int64
var debugOn bool
var Error float64
var pollInterval = float64(1000 / 400) // Time delta

var cpuTempLoad = 1
var fanTempReduction = 1

func main() {
  flag.StringVar(&thermalZone, "thermal-zone", "/sys/class/hwmon/hwmon4/temp1_input", "Path to CPU temperature reading")
  flag.StringVar(&ecPath, "ec-path", "/dev/ec", "Path to embedded controller interface")
  flag.Int64Var(&ecAddr, "ec-addr", 25, "Address for fan speed control register")
  flag.Int64Var(&manualAddr, "manual-addr", 21, "Address for manual control enable register")
  flag.Int64Var(&readAddr, "read-addr", 17, "Address for current speed register")
  flag.Float64Var(&ecMin, "ec-min", 0, "Minimum value to write to speed control register")
  flag.Float64Var(&ecMax, "ec-max", 59, "Maximum value to write to speed control register")
  flag.Int64Var(&readMin, "read-min", 14, "Minimum value that can be read from current speed register")
  flag.Int64Var(&readMax, "read-max", 54, "Maximum value that can be read from current speed register")
  flag.BoolVar(&debugOn, "debug", false, "Debug output")
  flag.Parse()

  fmt.Println("SmartFan V2 by petmshall")
  debug("Debug enabled")

  currSpeed = readSpeed()
  speedTarget = currSpeed

  checkManualControl()
  setupCloseHandler()

  loop()

  for range time.Tick(time.Duration(1000 / pollInterval) * time.Millisecond) {
    loop()
  }
}

func quit(msg error) {
  if (!gracefulQuitTried) {
    gracefulQuitTried = true
    disableManualControl()
  }
  panic(msg)
}

func debug(msg string) {
  if debugOn {
    fmt.Print("DEBUG: ")
    fmt.Println(msg)
  }
}

func loop() {
  currTemp = readTemp()
  updateSpeed()
  if !speedSatisfied {
    smoothSpeed()
  }
  writeSpeed()
}

func smoothSpeed() {
  if speedTarget > currSpeed {
    if speedTarget > 70 {
      currSpeed += 20
    } else if currSpeed > 60 {
      currSpeed += 10
    } else {
      currSpeed += 20
    }
    if speedTarget < currSpeed {
      currSpeed = speedTarget
      speedSatisfied = true
    }
  } else if speedTarget < currSpeed {
    currSpeed -= 5
    if speedTarget > currSpeed {
      currSpeed = speedTarget
      speedSatisfied = true
    }
  }
}

// Calculations

func updateSpeed() {
  if mode == 0 {
    // Passive cooling
    // debug("Passive cooling")
    if currTemp > highTemp {
      mode = 1
    }
    if currSpeed > 0 {
      speedTarget = 0
      speedSatisfied = false
    }
  } else {
    // Active cooling
    // debug("Active cooling")

    if currTemp < lowTemp {
      mode = 0
    }

    Error = currTemp - highTemp
    errorAccumulation = math.Max(math.Min(errorAccumulation + Error * pollInterval, 100), -400)

    if currTemp > oldTemp + tempRiseThreshold || currTemp + tempDropThreshold < oldTemp || currTemp >= throttleTemp {
      calcNewSpeed()
      oldTemp = currTemp
    }

    if speedTarget < minMode1Speed {
      speedTarget = minMode1Speed
      speedSatisfied = false
    }
  }
}

func calcNewSpeed() {
  // PID algorithm
  // highTemp is setpoint
  const P = float64(4.2)
  const I = float64(0.1)
  const D = float64(-2.3)
  // fmt.Println(errorAccumulation)
  var derivative = (Error - lastError) / pollInterval
  // fmt.Println(derivative)
  lastError = Error
  speedSatisfied = false
  if currTemp < highTemp {
    if currTemp > medTemp {
      speedTarget = math.Max(speedTarget, 40)
    } else {
      speedTarget = 20
    }
    return
  }
  speedTarget = Error * P + errorAccumulation * I + derivative * D
  // fmt.Println(currSpeed)
  if speedTarget < 20 {
    speedTarget = 20
  }
  if speedTarget > 100 {
    speedTarget = 100
  }
}

// Input / Output functions

func enableManualControl() {
  writeEC(manualAddr, 1)
}

func disableManualControl() {
  writeEC(manualAddr, 0)
}

func writeEC(address int64, value int) {
  f, err := os.OpenFile(ecPath, os.O_RDWR, 0644)
  if err != nil {
    quit(err)
  }
  defer f.Close()
  if _, err := f.WriteAt([]byte{byte(value)}, address); err != nil {
    quit(err)
  }
}

func readEC(address int64) int {
  f, err := os.OpenFile(ecPath, os.O_RDWR, 0644)
  if err != nil {
    quit(err)
  }
  defer f.Close()
  b := make([]byte, 1)
  f.ReadAt(b, address)
  return int(b[0])
}

func readTemp() float64 {
  f, err := ioutil.ReadFile(thermalZone)
  if err != nil {
    quit(err)
  }
  tempInt, _ := strconv.Atoi(strings.TrimRight(string(f), "\n"))
  return float64(tempInt) / 1000
}

func checkManualControl() {
  if readEC(manualAddr) == 0 {
    debug("Activating manual control")
    speedTarget = 0
    speedSatisfied = false
    mode = 0
    enableManualControl()
    writeSpeed()
  }
}

func readSpeed() float64 {
  ecVal := readEC(ecAddr)
  if debugOn {
    fmt.Println("Got EC value:", ecVal)
  }
  return math.Floor((float64(ecVal) - ecMin) / (ecMax - ecMin) * 100)
}

func writeSpeed() {
  ecVal := int(math.Floor(currSpeed / 100 * (ecMax - ecMin) + ecMin))
  if ecVal != lastECVal {
    lastECVal = ecVal
    writeEC(ecAddr, ecVal)
    if debugOn {
      fmt.Print("Speed: ", math.Round(currSpeed))
      fmt.Println("%, Wrote EC value:", ecVal)
    }
  }
}

// Process control

func setupCloseHandler() {
	c := make(chan os.Signal)
	signal.Notify(c, os.Interrupt, syscall.SIGTERM)
	go func() {
		<-c
		fmt.Println("\rHanding fan control over to EC")
    disableManualControl()
		os.Exit(0)
	}()
}
