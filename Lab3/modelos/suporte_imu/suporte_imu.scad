// Suporte da IMU (kit Arduino Uno + acrílico) para a pinça do UFACTORY Lite 6.
//
// A peça SUBSTITUI UM DEDO da pinça: prende-se à corrediça com os 2 parafusos
// M2,5 desse dedo, e o kit aparafusa-se à parede vertical pelos furos do
// Arduino Uno, com os MESMOS parafusos M3 que já lá estão (os dos espaçadores).
//
// Medidas da pinça: ficheiro STEP do fabricante (../gripper_lite/MEDIDAS.md).
// Eixos (os da pinça): X = largura do dedo, Y = direção em que os dedos abrem
// (positivo para fora), Z = para baixo, a partir da face inferior do corpo.
// Desenhada para a pinça FECHADA: a base do dedo começa em Y = 0, no eixo da
// ferramenta.
//
// Gerar:  openscad -o suporte_imu.stl suporte_imu.scad
// Imprimir com a face de fora da parede (onde encosta o acrílico) na mesa.

/* [Pinça: do STEP, não mexer sem medir] */
dedo_larg      = 20;     // X, largura da base do dedo
dedo_prof      = 15.3;   // Y, profundidade da base do dedo
furo_y         = 7.7;    // Y dos 2 parafusos, a contar da aresta de dentro
furo_dx        = 12;     // distância entre os 2 parafusos (X = ±6)
lamina_y       = 11.1;   // Y onde começava a lâmina do dedo original

/* [Parafusos da pinça (M2,5)] */
m25_passagem   = 2.9;    // furo de passagem
m25_cabeca     = 5.4;    // caixa da cabeça
sob_cabeca     = 2.0;    // plástico por baixo da cabeça. Original: 0,9 mm (parafuso M2,5x4).
                         // Com 2,0 mm usar M2,5x5; a rosca na corrediça tem 3 mm de fundo.
base_alt       = 7;      // Z, altura da base (igual à do dedo original)

/* [Parede e kit] */
parede_esp     = 5;      // Y
parede_larg    = 78;     // X
placa_topo     = 10;     // Z onde começa a placa do Arduino (aresta de cima)
folga_corpo    = 0.6;    // folga entre a parede e a face do corpo da pinça
m3_passagem    = 3.4;
m3_cabeca      = 6.6;    // caixa para a cabeça do M3, do lado de dentro
m3_sob_cabeca  = 1.6;    // plástico entre a cabeça do parafuso e o acrílico: é o que os
                         // parafusos que já estão no kit perdem de rosca no espaçador
m3_caixa       = parede_esp - m3_sob_cabeca;

/* [Reforços e cabo] */
nervura_esp    = 3;
nervura_alt    = 38;     // até onde descem as nervuras (Z)
rasgo          = [2.2, 5];   // rasgos para abraçadeira do cabo USB

$fn = 48;

// Furos de fixação do Arduino Uno (mm, placa de 68,6 x 53,3 vista de cima,
// fichas à esquerda). Como não sabemos para que lado fica virado o kit, a
// parede leva o padrão e o seu espelho: serve em qualquer das posições.
uno = [[14.0, 2.5], [15.2, 50.8], [66.0, 7.6], [66.0, 35.6]];
uno_larg = 68.6;  uno_alt = 53.3;
parede_alt = placa_topo + uno_alt + 4;         // Z onde acaba a parede

module furos_uno() {
    for (h = uno) for (s = [1, -1])
        translate([s * (h[0] - uno_larg / 2), 0, placa_topo + h[1]])
            rotate([-90, 0, 0]) children();
}

module base() {
    translate([-dedo_larg / 2, 0, 0]) cube([dedo_larg, dedo_prof, base_alt]);
}

module parede() {
    translate([-parede_larg / 2, lamina_y, folga_corpo])
        cube([parede_larg, parede_esp, parede_alt - folga_corpo]);
}

module nervuras() {
    for (s = [1, -1])
        translate([s * (dedo_larg / 2 + nervura_esp / 2), 0, 0])
            rotate([0, -90, 0]) translate([0, 0, -nervura_esp / 2])
                linear_extrude(nervura_esp)
                    polygon([[folga_corpo, 0], [folga_corpo, lamina_y + 0.1],
                             [nervura_alt, lamina_y + 0.1]]);
}

module suporte() {
    difference() {
        union() { base(); parede(); nervuras(); }
        // parafusos M2,5 da pinça: passagem e caixa da cabeça, por baixo
        for (s = [1, -1]) translate([s * furo_dx / 2, furo_y, 0]) {
            translate([0, 0, -1]) cylinder(d = m25_passagem, h = base_alt + 2);
            translate([0, 0, sob_cabeca]) cylinder(d = m25_cabeca, h = parede_alt);
        }
        // parafusos M3 do kit: passagem e caixa da cabeça do lado de dentro
        furos_uno() {
            translate([0, 0, lamina_y - 1]) cylinder(d = m3_passagem, h = parede_esp + 2);
            translate([0, 0, lamina_y - 20]) cylinder(d = m3_cabeca, h = 20 + m3_caixa);
        }
        // rasgos para a abraçadeira do cabo
        for (s = [1, -1]) for (dz = [0, 9])
            translate([s * (parede_larg / 2 - 4) - rasgo[0] / 2, lamina_y - 1, parede_alt - 22 + dz])
                cube([rasgo[0], parede_esp + 2, rasgo[1]]);
    }
}

// ---- contexto, só para ver (não entra no STL): pinça, o outro dedo e o kit
module contexto() {
    color("gainsboro", 0.5) translate([0, 0, -34]) cylinder(d = 77.5, h = 34, $fn = 96);
    color("dimgray") {                                   // o dedo que fica (fechado)
        translate([-10, -15.3, 0]) cube([20, 15.3, 7]);
        translate([-10, -15.3, 0]) cube([20, 4.2, 23.5]);
    }
    y0 = lamina_y + parede_esp;
    color("lightskyblue", 0.6) translate([-38, y0, placa_topo - 3]) cube([76, 3, uno_alt + 6]);      // acrílico
    color("royalblue") translate([-10, y0 + 3, placa_topo + 18]) cube([20, 3, 26]);                  // IMU
    color("seagreen") translate([-uno_larg / 2, y0 + 15, placa_topo]) cube([uno_larg, 1.6, uno_alt]); // Arduino
}

mostrar_contexto = false;
// vira-se o modelo para Z ficar para baixo no ecrã, como no robô
rotate([180, 0, 0]) {
    color("darkorange") suporte();
    if (mostrar_contexto) contexto();
}
