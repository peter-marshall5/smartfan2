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
  lowTemp = float64(45)
  medTemp = float64(62)
  highTemp = float64(70)
  tempSetpoint = float64(76)
  throttleTemp = float64(84)
  dangerousTemp = float64(94)
  tempRiseThreshold = float64(2)
  tempDropThreshold = float64(12)
  speedDropThreshold = float64(8)
  minMode1Speed = float64(34)
  P = float64(6)
  I = float64(0.17)
  D = float64(2.4)
)

var speedSatisfied = false
var speedTarget = float64(10)
var currSpeed = float64(0)
// 0 = passive, 1 = active
var mode = 1
var currTemp = float64(0)
var oldTemp = float64(0)
var lastECVal = 256
var errorAccumulation = float64(0)
var avgTemp = float64(100)
var lastError = float64(0)

var gracefulQuitTried = false

var thermalZone string
var ecPath string
var ecAddr int64
var manualAddr int64
var readAddr int64
var ecMin float64
var ecMax float64
var readMin float64
var readMax float64
var debugOn bool
var Error float64
var pollInterval = float64(400) // Time delta

var cpuTempLoad = 1
var fanTempReduction = 1

var manualControlTick = 0
var checkTempTick = 0

func main() {
  flag.StringVar(&thermalZone, "thermal-zone", "/sys/class/hwmon/hwmon4/temp1_input", "Path to CPU temperature reading")
  flag.StringVar(&ecPath, "ec-path", "/dev/ec", "Path to embedded controller interface")
  flag.Int64Var(&ecAddr, "ec-addr", 25, "Address for fan speed control register")
  flag.Int64Var(&manualAddr, "manual-addr", 21, "Address for manual control enable register")
  flag.Int64Var(&readAddr, "read-addr", 17, "Address for current speed register")
  flag.Float64Var(&ecMin, "ec-min", 0, "Minimum value to write to speed control register")
  flag.Float64Var(&ecMax, "ec-max", 59, "Maximum value to write to speed control register")
  flag.Float64Var(&readMin, "read-min", 4, "Minimum value that can be read from current speed register")
  flag.Float64Var(&readMax, "read-max", 59, "Maximum value that can be read from current speed register")
  flag.BoolVar(&debugOn, "debug", false, "Debug output")
  flag.Parse()

  fmt.Println("SmartFan V2 by petmshall")
  debug("Debug enabled")

  currSpeed = readSpeed()
  speedTarget = currSpeed

  enableManualControl()
  setupCloseHandler()

  loop()

  for range time.Tick(time.Duration(pollInterval) * time.Millisecond) {
    if manualControlTick > 10 {
      checkManualControl()
      manualControlTick = 0
    }
    if checkTempTick > 30 {
      if mode == 1 {
        calcNewSpeed()
        oldTemp = currTemp
      }
      checkTempTick = 0
    }
    manualControlTick++
    checkTempTick++
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
    fmt.Print("[INFO] ")
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
    if math.Abs(currSpeed - speedTarget) > 16 {
      currSpeed += 20
    } else if math.Abs(currSpeed - speedTarget) > 8 {
      currSpeed += 10
    } else {
      currSpeed += 4
    }
    if speedTarget < currSpeed {
      currSpeed = speedTarget
      speedSatisfied = true
    }
  } else if speedTarget < currSpeed {
    currSpeed -= 2
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
    if currTemp > medTemp {
      debug("Active cooling")
      mode = 1
      updateSpeed()
    }
    if currSpeed > 0 {
      speedTarget = 0
      speedSatisfied = false
    }
  } else {
    // Active cooling

    if avgTemp < lowTemp {
      debug("Passive cooling")
      mode = 0
    }

    Error = currTemp - tempSetpoint
    errorAccumulation = math.Max(math.Min(errorAccumulation + Error * (pollInterval / 1000), 500), -100)
    avgTemp += (currTemp - avgTemp) * (pollInterval / 1000) / 15

    if currTemp > oldTemp + tempRiseThreshold || currTemp + tempDropThreshold < oldTemp || currTemp >= throttleTemp {
      calcNewSpeed()
      oldTemp = currTemp
    }
  }
}

func calcNewSpeed() {
  // PID algorithm
  var newSpeed float64
  lastError = Error
  newSpeed = Error * P + errorAccumulation * I
  var derivative = (newSpeed - speedTarget) / (1000 / pollInterval)
  if derivative < 0 {
    derivative = 0
  }
  newSpeed -= derivative * D
  if debugOn {
    fmt.Print("[PID]  Error: ")
    fmt.Print(Error)
    fmt.Print(" Accumulation: ")
    fmt.Print(errorAccumulation)
    fmt.Print(" Derivative: ")
    fmt.Print(derivative)
    fmt.Print(" Speed: ")
    fmt.Println(newSpeed)
  }
  if newSpeed < minMode1Speed {
    newSpeed = minMode1Speed
  }
  if newSpeed > 100 {
    newSpeed = 100
  }
  // Reject speeds that are too similar
  if newSpeed > speedTarget || newSpeed < speedTarget - speedDropThreshold {
    if debugOn {
      fmt.Println("[PID]  New speed: ", newSpeed)
    }
    speedTarget = newSpeed
    speedSatisfied = false
    return
  }
}

// Input / Output functions

func enableManualControl() {
  writeEC(manualAddr, 1)
}

func disableManualControl() {
  writeEC(ecAddr, int(ecMax))
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
    enableManualControl()
    lastECVal = 256
    writeSpeed()
  }
}

func readSpeed() float64 {
  ecVal := readEC(readAddr)
  if debugOn {
    fmt.Println("[EC]   Read speed:", ecVal)
  }
  return math.Floor((float64(ecVal) - readMin) / (readMax - readMin) * 100)
}

func writeSpeed() {
  ecVal := int(math.Floor(currSpeed / 100 * (ecMax - ecMin) + ecMin))
  if ecVal != lastECVal {
    lastECVal = ecVal
    writeEC(ecAddr, ecVal)
    if debugOn {
      fmt.Print("[EC]   Wrote speed: ", math.Round(currSpeed))
      fmt.Println("% Value:", ecVal)
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
