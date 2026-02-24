const express = require('express');
const { spawn } = require('child_process');
const path = require('path');
const app = express();

app.use(express.json());
app.use(express.static('public'));
app.use('/outputs', express.static(path.join(__dirname, 'public', 'outputs')));

app.post('/calculate', (req, res) => {
    const { sx, sy, sz, tx, ty, tz, mode } = req.body;
    console.log("Hesaplama başladı:", req.body);

    res.setHeader('Content-Type', 'text/plain; charset=utf-8');
    res.setHeader('Transfer-Encoding', 'chunked');

    const python = spawn('python', [
        '-u',
        'app.py', 
        sx, sy, sz, tx, ty, tz, mode
    ]);

    python.stdout.on('data', (data) => {
        const output = data.toString();
        res.write(output);
    });

    python.stderr.on('data', (data) => {
        console.error(`Python Hata: ${data}`);
    });

    python.on('close', (code) => {
        console.log(`Python bitti. Kod: ${code}`);
        res.end();
    });
});

app.listen(3000, () => console.log('Sunucu Başladı: http://localhost:3000'));