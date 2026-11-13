// Placa para prender o kit da IMU (Arduino Uno + acrílico) à mão, com duas
// fitas de velcro de 25 mm. Aparafusa ao kit pelos furos do Arduino Uno, com
// os mesmos parafusos M3 que já lá estão (como o suporte da pinça).
// O lado liso encosta à mão (ou à luva); as cabeças dos parafusos ficam
// embutidas desse lado.
//
// Gerar:  openscad -o placa_mao.stl placa_mao.scad
// Imprimir com o lado da mão virado para a mesa.

larg          = 82;      // ao longo do lado comprido do Arduino
alt           = 64;
esp           = 3.2;
raio_canto    = 8;
m3_passagem   = 3.4;
m3_cabeca     = 6.6;
m3_sob_cabeca = 1.6;     // plástico entre a cabeça e o acrílico
fita_larg     = 27;      // rasgo para fita de 25 mm
fita_esp      = 3;
fita_margem   = 3.5;     // do rasgo à borda da placa

$fn = 48;
uno = [[13.97, 2.54], [15.24, 50.80], [66.04, 7.62], [66.04, 35.56]];
uno_larg = 68.58;  uno_alt = 53.34;

module contorno() {
    hull() for (sx = [-1, 1]) for (sy = [-1, 1])
        translate([sx * (larg / 2 - raio_canto), sy * (alt / 2 - raio_canto)]) circle(r = raio_canto);
}

difference() {
    linear_extrude(esp) contorno();
    // furos do Uno e o seu espelho: serve com o kit virado para qualquer lado
    for (h = uno) for (s = [1, -1]) translate([s * (h[0] - uno_larg / 2), h[1] - uno_alt / 2, 0]) {
        translate([0, 0, -1]) cylinder(d = m3_passagem, h = esp + 2);
        translate([0, 0, -1]) cylinder(d = m3_cabeca, h = 1 + esp - m3_sob_cabeca);   // cabeça embutida do lado da mão
    }
    // 4 rasgos para as fitas: duas fitas à volta da mão, uma em cada ponta da placa
    for (sx = [-1, 1]) for (sy = [-1, 1])
        translate([sx * (larg / 2 - fita_margem - fita_esp / 2), sy * (alt / 2 - 4 - fita_larg / 2), esp / 2])
            cube([fita_esp, fita_larg, esp + 2], center = true);
}
