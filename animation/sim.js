const simCanvas = document.getElementById("sim-canvas")
const chartCanvas = document.getElementById("chart-canvas")
const simContext = simCanvas.getContext("2d")
const chartContext = chartCanvas.getContext("2d")
const isEsp32 = document.body.dataset.mode === "esp32"

const angleValue = document.getElementById("angle-value")
const angleEstimate = document.getElementById("angle-estimate")
const positionValue = document.getElementById("position-value")
const positionEstimate = document.getElementById("position-estimate")
const forceValue = document.getElementById("force-value")
const statusValue = document.getElementById("status-value")
const timeValue = document.getElementById("time-value")
const controllerToggle = document.getElementById("controller-toggle")
const controllerState = document.getElementById("controller-state")
const pauseButton = document.getElementById("pause-button")
const resetButton = document.getElementById("reset-button")
const pushLeft = document.getElementById("push-left")
const pushRight = document.getElementById("push-right")
const angleInput = document.getElementById("angle-input")
const resetAngleValue = document.getElementById("reset-angle-value")
const forceInput = document.getElementById("force-input")
const forceLimitValue = document.getElementById("force-limit-value")
const speedInput = document.getElementById("speed-input")
const pausedLabel = document.getElementById("paused-label")
const connectButton = document.getElementById("connect-button")
const connectionDot = document.getElementById("connection-dot")
const connectionState = document.getElementById("connection-state")
const serialMessage = document.getElementById("serial-message")

//same model values as python simulation
const M = 1.0
const m = 0.1
const g = 9.81
const l = 0.5
const dt = 0.01

//same lqr gain as src/python/lqr.py and esp32
const K = [-3.16227766, -5.5397091, -56.84055809, -10.86364348]

//same discrete kalman model and noise values as src/python/kalman.py and esp32
const Ad = [
    [1.0, 0.01, -4.90588223e-5, -1.63517644e-7],
    [0.0, 1.0, -9.81352904e-3, -4.90588223e-5],
    [0.0, 0.0, 1.00107929, 1.00035974e-2],
    [0.0, 0.0, 2.15897639e-1, 1.00107929]
]

const Bd = [
    5.00008176e-5,
    1.00003270e-2,
    -1.00017986e-4,
    -2.00071948e-2
]

const Qk = [1e-6, 1e-4, 1e-6, 1e-4]
const positionStd = 0.002
const angleStd = degreesToRadians(0.2)

let state = [0, 0, degreesToRadians(5), 0]
let xHat = [0, 0, 0, 0]
let covariance = identityMatrix(4)
let measurement = [0, 0]
let uPrevious = 0
let controlForce = 0
let plantForce = 0
let externalForce = 0
let pushTime = 0
let simulationTime = 0
let running = true
let speed = 1
let uMax = Number(forceInput.value)
let accumulator = 0
let previousFrame = performance.now()
let history = []
let serialPort = null
let serialReader = null
let serialWriter = null
let serialBuffer = ""
let serialConnected = false
let connectionGeneration = 0

function degreesToRadians(value) {
    return value * Math.PI / 180
}

function radiansToDegrees(value) {
    return value * 180 / Math.PI
}

function clamp(value, minimum, maximum) {
    return Math.max(minimum, Math.min(maximum, value))
}

function identityMatrix(size) {
    return Array.from({ length: size }, (_, row) =>
        Array.from({ length: size }, (_, column) => row === column ? 1 : 0)
    )
}

function randomNormal() {
    let first = 0
    let second = 0

    while (first === 0) first = Math.random()
    while (second === 0) second = Math.random()

    return Math.sqrt(-2 * Math.log(first)) * Math.cos(2 * Math.PI * second)
}

function delay(milliseconds) {
    return new Promise(resolve => setTimeout(resolve, milliseconds))
}

function setSerialMessage(message, isError = false) {
    if (!serialMessage) return

    serialMessage.textContent = message
    serialMessage.classList.toggle("error", isError)
}

function setHardwareControls(enabled) {
    pauseButton.disabled = !enabled
    resetButton.disabled = !enabled
    pushLeft.disabled = !enabled
    pushRight.disabled = !enabled
}

function updateConnectionDisplay(connected, label) {
    if (!connectionState || !connectionDot || !connectButton) return

    connectionState.textContent = label
    connectionDot.classList.toggle("connected", connected)
    connectButton.textContent = connected ? "disconnect" : "connect esp32"
}

async function readEsp32Data(generation) {
    while (serialConnected && generation === connectionGeneration) {
        const newlineIndex = serialBuffer.indexOf("\n")

        if (newlineIndex !== -1) {
            const line = serialBuffer.slice(0, newlineIndex).trim()
            serialBuffer = serialBuffer.slice(newlineIndex + 1)
            const parts = line.split(",")

            if (parts.length === 6 && parts[0] === "DATA") {
                const values = parts.slice(1).map(Number)

                if (values.every(Number.isFinite)) {
                    return {
                        estimate: values.slice(0, 4),
                        force: values[4]
                    }
                }
            }

            continue
        }

        const { value, done } = await serialReader.read()

        if (done) {
            throw new Error("serial connection closed")
        }

        if (value) {
            serialBuffer += new TextDecoder().decode(value)
        }
    }

    return null
}

async function hardwareStep(generation) {
    measurement = [
        state[0] + randomNormal() * positionStd,
        state[2] + randomNormal() * angleStd
    ]

    const message = `MEAS,${measurement[0]},${measurement[1]}\n`
    await serialWriter.write(new TextEncoder().encode(message))

    const response = await readEsp32Data(generation)

    if (!response) return

    xHat = response.estimate
    controlForce = response.force

    if (pushTime > 0) {
        pushTime -= dt
    } else {
        externalForce = 0
    }

    plantForce = controlForce + externalForce

    const xDot = dynamics(state, plantForce)

    for (let i = 0; i < 4; i++) {
        state[i] += xDot[i] * dt
    }

    simulationTime += dt

    if (history.length === 0 || simulationTime - history[history.length - 1].time >= 0.04) {
        history.push({
            time: simulationTime,
            angle: radiansToDegrees(state[2]),
            estimateAngle: radiansToDegrees(xHat[2]),
            measurementAngle: radiansToDegrees(measurement[1]),
            force: controlForce
        })
    }

    while (history.length > 0 && simulationTime - history[0].time > 10) {
        history.shift()
    }
}

async function runHardwareLoop(generation) {
    try {
        while (serialConnected && generation === connectionGeneration) {
            if (running) {
                await hardwareStep(generation)
                await delay(dt * 1000 / speed)
            } else {
                await delay(40)
            }
        }
    } catch (error) {
        if (serialConnected && generation === connectionGeneration) {
            setSerialMessage(error.message || "serial communication failed", true)
            await closeSerialConnection()
        }
    }
}

async function openSerialConnection(selectedPort) {
    connectButton.disabled = true
    updateConnectionDisplay(false, "connecting")
    setSerialMessage("opening serial port and waiting for esp32 reset")

    try {
        serialPort = selectedPort
        await serialPort.open({ baudRate: 115200 })
        serialReader = serialPort.readable.getReader()
        serialWriter = serialPort.writable.getWriter()
        serialBuffer = ""
        await delay(2000)

        serialConnected = true
        connectionGeneration += 1
        resetSimulation()
        setHardwareControls(true)
        updateConnectionDisplay(true, "connected")
        setSerialMessage("live esp32 control · browser sends measurements and applies returned force")
        connectButton.disabled = false
        runHardwareLoop(connectionGeneration)
    } catch (error) {
        setSerialMessage(error.message || "could not open serial port", true)
        updateConnectionDisplay(false, "disconnected")
        connectButton.disabled = false
    }
}

async function closeSerialConnection(keepPort = false) {
    serialConnected = false
    connectionGeneration += 1
    running = false
    setHardwareControls(false)

    if (serialReader) {
        await serialReader.cancel().catch(() => {})
        serialReader.releaseLock()
        serialReader = null
    }

    if (serialWriter) {
        serialWriter.releaseLock()
        serialWriter = null
    }

    if (serialPort) {
        await serialPort.close().catch(() => {})
    }

    if (!keepPort) {
        serialPort = null
    }

    updateConnectionDisplay(false, "disconnected")
    connectButton.disabled = false
    pausedLabel.textContent = "waiting for esp32"
    pausedLabel.classList.remove("hidden")
}

async function connectEsp32() {
    if (!("serial" in navigator)) {
        setSerialMessage("web serial is not supported here · use chrome or edge on localhost", true)
        return
    }

    if (serialConnected) {
        await closeSerialConnection()
        setSerialMessage("esp32 disconnected")
        return
    }

    try {
        const selectedPort = await navigator.serial.requestPort()
        await openSerialConnection(selectedPort)
    } catch (error) {
        if (error.name !== "NotFoundError") {
            setSerialMessage(error.message || "serial port was not selected", true)
        }
    }
}

async function restartHardwareRun() {
    if (!serialPort || !serialConnected) return

    const selectedPort = serialPort
    connectButton.disabled = true
    await closeSerialConnection(true)
    await delay(250)
    await openSerialConnection(selectedPort)
}

function getTheme() {
    const styles = getComputedStyle(document.documentElement)

    return {
        text: styles.getPropertyValue("--text").trim(),
        muted: styles.getPropertyValue("--muted").trim(),
        line: styles.getPropertyValue("--line").trim(),
        surface: styles.getPropertyValue("--surface").trim(),
        surfaceRaised: styles.getPropertyValue("--surface-raised").trim(),
        green: styles.getPropertyValue("--green").trim(),
        purple: styles.getPropertyValue("--purple").trim(),
        orange: styles.getPropertyValue("--orange").trim(),
        red: styles.getPropertyValue("--red").trim()
    }
}

function lqrControl(x) {
    if (!controllerToggle.checked) {
        return 0
    }

    let result = 0

    for (let i = 0; i < 4; i++) {
        result += K[i] * x[i]
    }

    return clamp(-result, -uMax, uMax)
}

function dynamics(x, u) {
    const pDot = x[1]
    const theta = x[2]
    const thetaDot = x[3]
    const sinTheta = Math.sin(theta)
    const cosTheta = Math.cos(theta)
    const denominator = M + m - m * cosTheta * cosTheta

    const pDotDot = (
        u
        - m * g * sinTheta * cosTheta
        + m * l * thetaDot * thetaDot * sinTheta
    ) / denominator

    const thetaDotDot = (g * sinTheta - pDotDot * cosTheta) / l

    return [pDot, pDotDot, thetaDot, thetaDotDot]
}

function kalmanStep(estimate, P, u, y) {
    const prediction = [0, 0, 0, 0]

    for (let row = 0; row < 4; row++) {
        prediction[row] = Bd[row] * u

        for (let column = 0; column < 4; column++) {
            prediction[row] += Ad[row][column] * estimate[column]
        }
    }

    const AP = Array.from({ length: 4 }, () => Array(4).fill(0))
    const predictedP = Array.from({ length: 4 }, () => Array(4).fill(0))

    for (let row = 0; row < 4; row++) {
        for (let column = 0; column < 4; column++) {
            for (let index = 0; index < 4; index++) {
                AP[row][column] += Ad[row][index] * P[index][column]
            }
        }
    }

    for (let row = 0; row < 4; row++) {
        for (let column = 0; column < 4; column++) {
            for (let index = 0; index < 4; index++) {
                predictedP[row][column] += AP[row][index] * Ad[column][index]
            }

            if (row === column) {
                predictedP[row][column] += Qk[row]
            }
        }
    }

    //C measures position and angle so the 2 by 2 inverse can stay simple
    const s00 = predictedP[0][0] + positionStd * positionStd
    const s01 = predictedP[0][2]
    const s10 = predictedP[2][0]
    const s11 = predictedP[2][2] + angleStd * angleStd
    const determinant = s00 * s11 - s01 * s10
    const gain = Array.from({ length: 4 }, () => [0, 0])

    for (let row = 0; row < 4; row++) {
        gain[row][0] = (predictedP[row][0] * s11 - predictedP[row][2] * s10) / determinant
        gain[row][1] = (-predictedP[row][0] * s01 + predictedP[row][2] * s00) / determinant
    }

    const residual = [y[0] - prediction[0], y[1] - prediction[2]]
    const corrected = prediction.map((value, row) =>
        value + gain[row][0] * residual[0] + gain[row][1] * residual[1]
    )
    const correctedP = Array.from({ length: 4 }, () => Array(4).fill(0))

    for (let row = 0; row < 4; row++) {
        for (let column = 0; column < 4; column++) {
            correctedP[row][column] = predictedP[row][column]
                - gain[row][0] * predictedP[0][column]
                - gain[row][1] * predictedP[2][column]
        }
    }

    return [corrected, correctedP]
}

function stepSimulation() {
    //the controller only sees these noisy measurements through the kalman filter
    measurement = [
        state[0] + randomNormal() * positionStd,
        state[2] + randomNormal() * angleStd
    ]

    ;[xHat, covariance] = kalmanStep(xHat, covariance, uPrevious, measurement)
    controlForce = lqrControl(xHat)

    if (pushTime > 0) {
        pushTime -= dt
    } else {
        externalForce = 0
    }

    plantForce = controlForce + externalForce

    const xDot = dynamics(state, plantForce)

    for (let i = 0; i < 4; i++) {
        state[i] += xDot[i] * dt
    }

    uPrevious = controlForce
    simulationTime += dt

    if (history.length === 0 || simulationTime - history[history.length - 1].time >= 0.04) {
        history.push({
            time: simulationTime,
            angle: radiansToDegrees(state[2]),
            estimateAngle: radiansToDegrees(xHat[2]),
            measurementAngle: radiansToDegrees(measurement[1]),
            force: controlForce
        })
    }

    while (history.length > 0 && simulationTime - history[0].time > 10) {
        history.shift()
    }
}

function resetSimulation() {
    state = [0, 0, degreesToRadians(Number(angleInput.value)), 0]
    xHat = [0, 0, 0, 0]
    covariance = identityMatrix(4)
    measurement = [state[0], state[2]]
    uPrevious = 0
    controlForce = 0
    plantForce = 0
    externalForce = 0
    pushTime = 0
    simulationTime = 0
    accumulator = 0
    history = []
    running = !isEsp32 || serialConnected
    pauseButton.textContent = "pause"
    pausedLabel.textContent = running ? "paused" : "waiting for esp32"
    pausedLabel.classList.toggle("hidden", running)
    updateReadouts()
}

function applyPush(direction) {
    externalForce = direction * 8
    pushTime = 0.18
}

function resizeCanvas(canvas, context) {
    const ratio = window.devicePixelRatio || 1
    const bounds = canvas.getBoundingClientRect()
    const width = Math.round(bounds.width * ratio)
    const height = Math.round(bounds.height * ratio)

    if (canvas.width !== width || canvas.height !== height) {
        canvas.width = width
        canvas.height = height
        context.setTransform(ratio, 0, 0, ratio, 0, 0)
    }

    return bounds
}

function roundRect(context, x, y, width, height, radius) {
    const r = Math.min(radius, width / 2, height / 2)

    context.beginPath()
    context.moveTo(x + r, y)
    context.arcTo(x + width, y, x + width, y + height, r)
    context.arcTo(x + width, y + height, x, y + height, r)
    context.arcTo(x, y + height, x, y, r)
    context.arcTo(x, y, x + width, y, r)
    context.closePath()
}

function drawArrow(context, startX, endX, y, color) {
    if (Math.abs(endX - startX) < 4) {
        return
    }

    const direction = Math.sign(endX - startX)

    context.save()
    context.strokeStyle = color
    context.fillStyle = color
    context.lineWidth = 2
    context.beginPath()
    context.moveTo(startX, y)
    context.lineTo(endX, y)
    context.stroke()
    context.beginPath()
    context.moveTo(endX, y)
    context.lineTo(endX - direction * 8, y - 5)
    context.lineTo(endX - direction * 8, y + 5)
    context.closePath()
    context.fill()
    context.restore()
}

function drawSimulation() {
    const bounds = resizeCanvas(simCanvas, simContext)
    const width = bounds.width
    const height = bounds.height
    const theme = getTheme()
    const railY = height * 0.72
    const horizontalMargin = Math.max(42, width * 0.07)
    const pixelsPerMeter = (width - horizontalMargin * 2) / 4.8
    const cartX = width / 2 + state[0] * pixelsPerMeter
    const estimateCartX = width / 2 + xHat[0] * pixelsPerMeter
    const cartWidth = clamp(width * 0.1, 72, 104)
    const cartHeight = clamp(height * 0.085, 28, 42)
    const wheelRadius = clamp(height * 0.022, 7, 11)
    const poleLength = clamp(height * 0.43, 130, 205)
    const pivotY = railY - cartHeight - 4
    const poleEndX = cartX + Math.sin(state[2]) * poleLength
    const poleEndY = pivotY - Math.cos(state[2]) * poleLength
    const estimatePoleEndX = estimateCartX + Math.sin(xHat[2]) * poleLength
    const estimatePoleEndY = pivotY - Math.cos(xHat[2]) * poleLength

    simContext.clearRect(0, 0, width, height)

    //center reference
    simContext.save()
    simContext.strokeStyle = theme.line
    simContext.setLineDash([4, 7])
    simContext.beginPath()
    simContext.moveTo(width / 2, 24)
    simContext.lineTo(width / 2, railY + 28)
    simContext.stroke()
    simContext.restore()

    //track
    simContext.strokeStyle = theme.muted
    simContext.lineWidth = 2
    simContext.beginPath()
    simContext.moveTo(horizontalMargin, railY)
    simContext.lineTo(width - horizontalMargin, railY)
    simContext.stroke()

    simContext.strokeStyle = theme.line
    simContext.lineWidth = 1

    for (let x = horizontalMargin; x <= width - horizontalMargin; x += pixelsPerMeter * 0.2) {
        simContext.beginPath()
        simContext.moveTo(x, railY + 7)
        simContext.lineTo(x, railY + 14)
        simContext.stroke()
    }

    //kalman estimate shown behind the hidden true system
    simContext.save()
    simContext.strokeStyle = theme.purple
    simContext.fillStyle = theme.purple
    simContext.globalAlpha = 0.72
    simContext.lineWidth = 3
    simContext.setLineDash([7, 6])
    simContext.beginPath()
    simContext.moveTo(estimateCartX, pivotY)
    simContext.lineTo(estimatePoleEndX, estimatePoleEndY)
    simContext.stroke()
    simContext.setLineDash([])
    simContext.beginPath()
    simContext.arc(estimatePoleEndX, estimatePoleEndY, 5, 0, Math.PI * 2)
    simContext.fill()
    simContext.restore()

    //force arrow
    const forceScale = clamp(width * 0.012, 7, 13)
    drawArrow(
        simContext,
        cartX,
        cartX + plantForce * forceScale,
        pivotY + cartHeight * 0.45,
        externalForce === 0 ? theme.green : theme.orange
    )

    //pole glow
    simContext.save()
    simContext.strokeStyle = theme.green
    simContext.lineWidth = 13
    simContext.globalAlpha = 0.08
    simContext.beginPath()
    simContext.moveTo(cartX, pivotY)
    simContext.lineTo(poleEndX, poleEndY)
    simContext.stroke()
    simContext.restore()

    //pole
    simContext.strokeStyle = theme.text
    simContext.lineWidth = 7
    simContext.lineCap = "round"
    simContext.beginPath()
    simContext.moveTo(cartX, pivotY)
    simContext.lineTo(poleEndX, poleEndY)
    simContext.stroke()

    //top mass
    simContext.fillStyle = theme.green
    simContext.beginPath()
    simContext.arc(poleEndX, poleEndY, 8, 0, Math.PI * 2)
    simContext.fill()

    //cart
    simContext.fillStyle = theme.surfaceRaised
    simContext.strokeStyle = theme.muted
    simContext.lineWidth = 1.5
    roundRect(
        simContext,
        cartX - cartWidth / 2,
        pivotY,
        cartWidth,
        cartHeight,
        7
    )
    simContext.fill()
    simContext.stroke()

    //pivot
    simContext.fillStyle = theme.green
    simContext.beginPath()
    simContext.arc(cartX, pivotY, 7, 0, Math.PI * 2)
    simContext.fill()

    //wheels
    const wheelY = pivotY + cartHeight + wheelRadius * 0.3

    for (const wheelX of [cartX - cartWidth * 0.3, cartX + cartWidth * 0.3]) {
        simContext.fillStyle = theme.surface
        simContext.strokeStyle = theme.muted
        simContext.lineWidth = 2
        simContext.beginPath()
        simContext.arc(wheelX, wheelY, wheelRadius, 0, Math.PI * 2)
        simContext.fill()
        simContext.stroke()
        simContext.fillStyle = theme.muted
        simContext.beginPath()
        simContext.arc(wheelX, wheelY, 2.5, 0, Math.PI * 2)
        simContext.fill()
    }

    //out of range marker
    if (Math.abs(state[0]) > 2.4) {
        simContext.fillStyle = theme.red
        simContext.font = "11px DM Mono"
        simContext.textAlign = state[0] > 0 ? "right" : "left"
        simContext.fillText(
            "cart outside track",
            state[0] > 0 ? width - 18 : 18,
            52
        )
    }
}

function drawChart() {
    const bounds = resizeCanvas(chartCanvas, chartContext)
    const width = bounds.width
    const height = bounds.height
    const theme = getTheme()
    const padding = { top: 16, right: 48, bottom: 23, left: 38 }
    const plotWidth = width - padding.left - padding.right
    const plotHeight = height - padding.top - padding.bottom
    const largestAngle = history.reduce((largest, point) => Math.max(
        largest,
        Math.abs(point.angle),
        Math.abs(point.estimateAngle),
        Math.abs(point.measurementAngle)
    ), 0)
    const angleLimit = Math.max(20, Math.min(50, Math.ceil(largestAngle / 10) * 10))

    chartContext.clearRect(0, 0, width, height)
    chartContext.font = "10px DM Mono"
    chartContext.textBaseline = "middle"

    //grid and labels
    for (let i = 0; i <= 4; i++) {
        const y = padding.top + plotHeight * i / 4
        const angleTick = angleLimit - i * angleLimit / 2
        const forceTick = uMax - i * uMax / 2

        chartContext.strokeStyle = theme.line
        chartContext.lineWidth = 1
        chartContext.beginPath()
        chartContext.moveTo(padding.left, y)
        chartContext.lineTo(width - padding.right, y)
        chartContext.stroke()

        chartContext.fillStyle = theme.muted
        chartContext.textAlign = "right"
        chartContext.fillText(`${angleTick}°`, padding.left - 8, y)
        chartContext.textAlign = "left"
        chartContext.fillText(`${Number(forceTick.toFixed(1))} N`, width - padding.right + 8, y)
    }

    if (history.length < 2) {
        chartContext.fillStyle = theme.muted
        chartContext.textAlign = "center"
        chartContext.fillText("collecting response data", width / 2, height / 2)
        return
    }

    const newestTime = history[history.length - 1].time
    const oldestTime = Math.max(0, newestTime - 10)
    const timeRange = Math.max(10, newestTime - oldestTime)

    function xPosition(time) {
        return padding.left + ((time - oldestTime) / timeRange) * plotWidth
    }

    function anglePosition(angle) {
        return padding.top + (1 - (clamp(angle, -angleLimit, angleLimit) + angleLimit) / (2 * angleLimit)) * plotHeight
    }

    function forcePosition(force) {
        return padding.top + (1 - (clamp(force, -uMax, uMax) + uMax) / (2 * uMax)) * plotHeight
    }

    function drawSeries(getY, color, lineWidth, dash = [], opacity = 1) {
        chartContext.save()
        chartContext.strokeStyle = color
        chartContext.lineWidth = lineWidth
        chartContext.lineJoin = "round"
        chartContext.globalAlpha = opacity
        chartContext.setLineDash(dash)
        chartContext.beginPath()

        history.forEach((point, index) => {
            const x = xPosition(point.time)
            const y = getY(point)

            if (index === 0) {
                chartContext.moveTo(x, y)
            } else {
                chartContext.lineTo(x, y)
            }
        })

        chartContext.stroke()
        chartContext.restore()
    }

    //force uses its own right hand scale and stays behind the angle traces
    drawSeries(point => forcePosition(point.force), theme.orange, 1.25, [3, 4], 0.7)

    //raw noisy angle measurement dots
    chartContext.save()
    chartContext.fillStyle = theme.muted
    chartContext.globalAlpha = 0.5

    history.forEach(point => {
        chartContext.beginPath()
        chartContext.arc(xPosition(point.time), anglePosition(point.measurementAngle), 1.3, 0, Math.PI * 2)
        chartContext.fill()
    })

    chartContext.restore()

    //soft actual reference underneath with kalman estimate clearly on top
    drawSeries(point => anglePosition(point.angle), theme.green, 4, [], 0.55)
    drawSeries(point => anglePosition(point.estimateAngle), theme.purple, 2.5, [7, 5])

    chartContext.fillStyle = theme.muted
    chartContext.textAlign = "left"
    chartContext.fillText(`${oldestTime.toFixed(1)} s`, padding.left, height - 9)
    chartContext.textAlign = "right"
    chartContext.fillText(`${newestTime.toFixed(1)} s`, width - padding.right, height - 9)
}

function updateReadouts() {
    const angle = radiansToDegrees(state[2])
    const absoluteAngle = Math.abs(angle)
    const estimateName = isEsp32 ? "esp32" : "kalman"

    angleValue.textContent = `${angle.toFixed(2)}°`
    angleEstimate.textContent = `${estimateName} ${radiansToDegrees(xHat[2]).toFixed(2)}°`
    positionValue.textContent = `${state[0].toFixed(3)} m`
    positionEstimate.textContent = `${estimateName} ${xHat[0].toFixed(3)} m`
    forceValue.textContent = `${controlForce.toFixed(2)} N`
    timeValue.textContent = `t ${simulationTime.toFixed(2)} s`

    statusValue.className = "status"

    if (isEsp32 && !serialConnected) {
        statusValue.textContent = "waiting"
        statusValue.classList.add("waiting")
        return
    }

    if (absoluteAngle < 0.5 && Math.abs(state[3]) < 0.08) {
        statusValue.textContent = "stable"
        statusValue.classList.add("stable")
    } else if (absoluteAngle < 20) {
        statusValue.textContent = "settling"
        statusValue.classList.add("settling")
    } else {
        statusValue.textContent = "falling"
        statusValue.classList.add("falling")
    }
}

function animate(currentFrame) {
    const elapsed = Math.min((currentFrame - previousFrame) / 1000, 0.1)
    previousFrame = currentFrame

    if (running && !isEsp32) {
        accumulator += elapsed * speed

        while (accumulator >= dt) {
            stepSimulation()
            accumulator -= dt
        }
    }

    drawSimulation()
    drawChart()
    updateReadouts()
    requestAnimationFrame(animate)
}

controllerToggle.addEventListener("change", () => {
    controllerState.textContent = controllerToggle.checked ? "on" : "off"
    controllerState.style.color = controllerToggle.checked ? "var(--green)" : "var(--red)"
})

pauseButton.addEventListener("click", () => {
    running = !running
    pauseButton.textContent = running ? "pause" : "resume"
    pausedLabel.classList.toggle("hidden", running)
})

resetButton.addEventListener("click", () => {
    if (isEsp32) {
        restartHardwareRun()
    } else {
        resetSimulation()
    }
})
pushLeft.addEventListener("click", () => applyPush(-1))
pushRight.addEventListener("click", () => applyPush(1))

angleInput.addEventListener("input", () => {
    resetAngleValue.textContent = `${Number(angleInput.value)}°`
})

forceInput.addEventListener("input", () => {
    uMax = Number(forceInput.value)
    forceLimitValue.textContent = `${uMax} N`
})

speedInput.addEventListener("change", () => {
    speed = Number(speedInput.value)
})

if (connectButton) {
    connectButton.addEventListener("click", connectEsp32)
}

window.addEventListener("resize", () => {
    drawSimulation()
    drawChart()
})

resetSimulation()

if (isEsp32) {
    setHardwareControls(false)
    updateConnectionDisplay(false, "disconnected")

    if (!("serial" in navigator)) {
        setSerialMessage("web serial is not supported here · use chrome or edge on localhost", true)
        connectButton.disabled = true
    }
}

requestAnimationFrame(animate)
