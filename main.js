

function setup()
{
    createCanvas(950, 950);
}

function move(v, t)
{
    return { x: v.x + t.x,
             y: v.y + t.y,
             z: v.z + t.z };
}

function rot(vec, axis, angle)
{
    let x = vec.x;
    let y = vec.y;
    let z = vec.z;

    let u = axis.x;
    let v = axis.y;
    let w = axis.z;

    let xPrime = u*(u*x + v*y + w*z)*(1 - Math.cos(angle)) 
                + x*Math.cos(angle)
                    + (-w*y + v*z)*Math.sin(angle);
    let yPrime = v*(u*x + v*y + w*z)*(1 - Math.cos(angle))
                    + y*Math.cos(angle)
                    + (w*x - u*z)*Math.sin(angle);
    let zPrime = w*(u*x + v*y + w*z)*(1 - Math.cos(angle))
                    + z*Math.cos(angle)
                    + (-v*x + u*y)*Math.sin(angle);

    return { x: xPrime, y: yPrime, z: zPrime };
}

let cameraPos = { x: 0, y: 0, z: 5 };
let cameraRot = 0;

function drawPoints(points)
{
    for (let bp of points)
    {
        let p = bp;
        p = rot(p, { x: 0, y: 1, z: 0}, cameraRot);
        p = move(p, cameraPos);

        if (p.z <= 0) continue;

        let x = p.x / p.z * width / 2 + width / 2;
        let y = -p.y / p.z * height / 2 + height / 2;

        // strokeWeight(10 / p.z);
        // point(x, y);

        for (let bq of points)
        {
            let q = bq;
            q = rot(q, { x: 0, y: 1, z: 0}, cameraRot);
            q = move(q, cameraPos);
            let x_ = q.x / q.z * width / 2 + width / 2;
            let y_ = -q.y / q.z * height / 2 + height / 2;
            if (q.z <= 0) continue;

            if ((q.x - p.x) * (q.x - p.x) + (q.y - p.y) * (q.y - p.y) + (q.z - p.z) * (q.z - p.z) == 0)
                continue;

            let w = 0.2 / ( (q.x - p.x) * (q.x - p.x) + (q.y - p.y) * (q.y - p.y) + (q.z - p.z) * (q.z - p.z) );
            w = min(w, 5);
            strokeWeight(w  / (q.z / 2 + p.z / 2)  );
            line(x, y, x_, y_);
        }

    }
}



let keyState = {};

function keyPressed()
{
    keyState[key] = true;
}

function keyReleased()
{
    keyState[key] = false;
}

function k(x)
{
    if (Object.keys(keyState).includes(x))
        return keyState[x];
    return false;
}


// slam

function applyTransform(p, t)
{
    p = move(p, t.pos);
    p = rot(p, { x:0, y:1, z:0 }, t.rot.y);
    p = rot(p, { x:0, y:0, z:1 }, t.rot.z);
    p = rot(p, { x:1, y:0, z:0 }, t.rot.x);
    return p;
}

function cost(a, b)
{
    return  (a.x - b.x) * (a.x - b.x) +
            (a.y - b.y) * (a.y - b.y) +
            (a.z - b.z) * (a.z - b.z);
}

function costSum(a, b, t)
{
    let error = 0;
    for (let i = 0; i < a.length; i++)
    {
        let nex = a[i];
        let old = applyTransform(b[i], t);

        error += cost(nex, old) / a.length;
    }
    return error;
}

function reverseEngineer(prime, prior)
{
    let eps = 0.001;
    let rate = 0.005;
    let base = costSum(prime, prior, { pos: { x : 0, y : 0, z : 0 }, rot : { x : 0, y: 0, z: 0 } });
    let grad = {
        x:  (costSum(prime, prior, { pos: { x : eps, y : 0, z : 0 }, rot : { x : 0, y: 0, z: 0 } }) - base) / eps,
        y:  (costSum(prime, prior, { pos: { x : 0, y : eps, z : 0 }, rot : { x : 0, y: 0, z: 0 } }) - base) / eps,
        z:  (costSum(prime, prior, { pos: { x : 0, y : 0, z : eps }, rot : { x : 0, y: 0, z: 0 } }) - base) / eps,

        rx: (costSum(prime, prior, { pos: { x : 0, y : 0, z : 0 }, rot : { x : eps, y: 0, z: 0 } }) - base) / eps,
        ry: (costSum(prime, prior, { pos: { x : 0, y : 0, z : 0 }, rot : { x : 0, y: eps, z: 0 } }) - base) / eps,
        rz: (costSum(prime, prior, { pos: { x : 0, y : 0, z : 0 }, rot : { x : 0, y: 0, z: eps } }) - base) / eps,
    };

    return {
        x: grad.x * rate,
        y: grad.y * rate,
        z: grad.z * rate,

        rx: grad.rx * rate,
        ry: grad.ry * rate,
        rz: grad.rz * rate,
    };
}



let points = [];
let pointsNoise = [];

let pointsPrime = [];

let interpPoints = [];
let interpPrimes = [];

let groundTruth = {
    pos: { x: 0, y: 0, z: 0 },
    rot: { x: 0, y: 1, z: 0 },
};
let answer = groundTruth;

let lastKeys = {};

let avgStdDev = 0;
let stdDevSamples = 0;

let lastBase = 0;

let iq = 0;
function draw()
{
    background(15);
    strokeWeight(2);

    for (let i = 0; i < points.length; i++)
        interpPoints[i] = {
            x: lerp(points[i].x, interpPoints[i].x, 0.9),
            y: lerp(points[i].y, interpPoints[i].y, 0.9),
            z: lerp(points[i].z, interpPoints[i].z, 0.9),
        };
    for (let i = 0; i < pointsPrime.length; i++)
        interpPrimes[i] = {
            x: lerp(pointsPrime[i].x, interpPrimes[i].x, 0),
            y: lerp(pointsPrime[i].y, interpPrimes[i].y, 0),
            z: lerp(pointsPrime[i].z, interpPrimes[i].z, 0),
        };


    stroke(0,100,255);
    drawPoints(interpPoints);


    // {
    //     stroke(255,0,0,100);
    //     let noiseless = [];
    //     for (let i = 0; i < points.length; i++)
    //     {
    //         let point = points[i];
    //         noiseless.push(applyTransform(point, groundTruth));
    //     }
    //     drawPoints(noiseless);
    // }
    stroke(255,0,100);
    drawPoints(interpPrimes);


    for (let i = 0; i < 10; i++)
    {
        let diff = reverseEngineer(points, pointsPrime);
        // groundTruth.pos.x -= diff.x;
        // groundTruth.pos.y -= diff.y;
        // groundTruth.pos.z -= diff.z;

        // groundTruth.rot.x -= diff.rx;
        // groundTruth.rot.y -= diff.ry;
        // groundTruth.rot.z -= diff.rz;

        for (let j = 0; j < pointsPrime.length; j++)
        {
            pointsPrime[j] = applyTransform(pointsPrime[j], { pos: { x: -diff.x, y: -diff.y, z: -diff.z }, rot: { x: -diff.rx, y: -diff.ry, z: -diff.rz } });
        }
    }

    let base = costSum(pointsPrime, points, { pos: { x : 0, y : 0, z : 0 }, rot : { x : 0, y: 0, z: 0 } });
    // console.log(base);

    cameraRot += (k('e') - k('q')) * 0.1;

    cameraPos.x += (k('a') - k('d')) * 0.1;
    cameraPos.y += (k("Shift") - k(' ')) * 0.1;
    cameraPos.z += (k('s') - k('w')) * 0.1;


    groundTruth.rot.y += (k('t') - k('y')) * 0.01;



    //if (!k('p') && lastKeys['p'])
    if (lastBase - base < 0.000001 && base < 0.1)
    {
        
        points = [];
        pointsNoise = [];
        for (let i = 0; i < 50; i++)
        {
            points.push({ x: Math.random() * 2 - 1, y: Math.random() * 2 - 1, z: Math.random() * 2 - 1 });
            pointsNoise[i] = { x: points[i].x, y: points[i].y, z: points[i].z };
            
            let n = Math.random() * 0.1;

            pointsNoise[i].x += (Math.random() * 2 - 1) * n;
            pointsNoise[i].y += (Math.random() * 2 - 1) * n;
            pointsNoise[i].z += (Math.random() * 2 - 1) * n;

            if (interpPoints.length <= i) interpPoints.push({x:0,y:0,z:0});
        }
        

        groundTruth = {
            pos: { x: Math.random() * 20 - 10, y : Math.random() * 20 - 10, z : Math.random() * 20 - 10 },
            rot: { x: Math.random() * Math.PI * 3 - Math.PI / 2, y : Math.random() * Math.PI * 3 - Math.PI / 2, z : Math.random() * Math.PI * 3 - Math.PI / 2},
        };

        pointsPrime = [];
        interpPrimes = [];
        for (let i = 0; i < points.length; i++)
        {
            let point = pointsNoise[i];
            pointsPrime.push(applyTransform(point, groundTruth));

            interpPrimes[i] = pointsPrime[i];
        }
    }

    //     console.log("---");
    //     // console.log("dx:"+  (answer.pos.x - groundTruth.pos.x));
    //     // console.log("dy:"+  (answer.pos.y - groundTruth.pos.y));
    //     // console.log("dz:"+  (answer.pos.z - groundTruth.pos.z));
    //     // console.log("drx:"+ (answer.rot.x - groundTruth.rot.x));
    //     // console.log("dry:"+ (answer.rot.y - groundTruth.rot.y));
    //     // console.log("drz:"+ (answer.rot.z - groundTruth.rot.z));


    //     let variance =
    //     ((groundTruth.pos.x) * (groundTruth.pos.x) +
    //      (groundTruth.pos.y) * (groundTruth.pos.y) +
    //      (groundTruth.pos.z) * (groundTruth.pos.z)) / 3;


    //     let rot_variance =
    //      (groundTruth.rot.x) * (groundTruth.rot.x) +
    //      (groundTruth.rot.y) * (groundTruth.rot.y) +
    //      (groundTruth.rot.z) * (groundTruth.rot.z);
    //      rot_variance /= 3;

    //     console.log("stddev: " + sqrt(variance));
    //     console.log("rot stddev: " + sqrt(rot_variance));
        
    //     if (iq > 0)
    //     {
    //         avgStdDev = avgStdDev * (stdDevSamples / (stdDevSamples + 1)) + sqrt(variance) / (stdDevSamples + 1);
    //         stdDevSamples ++;
    //     }

    //     console.log("avg: " + avgStdDev);

    //     groundTruth = {
    //         pos: { x: Math.random() * 5 - 2.5, y : Math.random() * 5 - 2.5, z : Math.random() * 5 - 2.5 },
    //         rot: { x: Math.random() * Math.PI - Math.PI / 2, y : Math.random() * Math.PI - Math.PI / 2, z : Math.random() * Math.PI - Math.PI / 2},
    //     };

    //     answer = JSON.parse(JSON.stringify(groundTruth));

    //     lastBase = 0;

    //     iq++;
    // }

    lastBase = base;
    for (let keys in keyState) lastKeys[keys] = keyState[keys];
}