// Suporte da IMU para a pinça do UFACTORY Lite 6 (peça "1").
//
// A peça SUBSTITUI UM DEDO da pinça: prende-se ao carro com os 2 parafusos
// M2,5 desse dedo e abraça o carro com dois rebordos, como o dedo original.
// Na parede aparafusa-se o Arduino Uno, sem o acrílico: as torres de
// espaçadores do kit (16 mm) encostam à parede e o parafuso M3 entra por
// dentro, com a cabeça à vista. A placa Grove da IMU (40 x 20 mm) fica debaixo
// do Uno, apertada pela aresta por 3 parafusos M3 x 10; só a ponta com a ficha
// Grove sai para lá da aresta da parede (o cabo entra na perpendicular à placa
// e não caberia debaixo do Uno). A posição do chip fica conhecida pelo desenho.
//
// Medidas da pinça: ficheiro STEP do fabricante (../gripper_lite/MEDIDAS.md),
// confirmadas no laboratório a 16/11. Placa da IMU: ficheiro Eagle da Seeed
// (Grove - IMU 9DOF ICM20600 & AK09918) e medidas no laboratório. Furos do Uno:
// desenho de referência da Arduino.
// Eixos (os da pinça): X = largura do dedo, Y = direção em que os dedos abrem
// (positivo para fora), Z = para baixo, a partir da face inferior do corpo
// (onde assenta o dedo; os rebordos sobem 1,5 mm à volta do carro).
//
// Gerar:  openscad --export-format binstl -o suporte_imu.stl suporte_imu.scad
// Imprimir com a face de fora da parede (a da IMU e do Uno) na mesa.

/* [Pinça: do STEP, não mexer sem medir] */
dedo_larg      = 20;     // X, largura da base do dedo
dedo_prof      = 15.3;   // Y, profundidade da base do dedo
furo_y         = 7.7;    // Y dos 2 parafusos, a contar da aresta de dentro
furo_dx        = 12;     // distância entre os 2 parafusos (X = ±6)
lamina_y       = 11.1;   // Y onde começava a lâmina do dedo original
carro_larg     = 17;     // X, largura do carro (medido): os rebordos abraçam-no
rebordo        = [1.5, 1.5];   // espessura (X) e altura (Z, para cima) dos rebordos
folga_carro    = 0.2;    // de cada lado, entre o rebordo e o carro

/* [Parafusos da pinça (M2,5 x 4, os do dedo)] */
m25_passagem   = 3.0;    // furo de passagem
m25_cabeca     = 5.4;    // caixa da cabeça (cabeça medida: Ø 4,5 x 2,5)
sob_cabeca     = 1.2;    // plástico por baixo da cabeça: ficam 2,8 mm de rosca no carro
base_alt       = 7;      // Z, altura da base

/* [Parede e Arduino] */
parede_esp     = 3;      // Y: com o parafuso M3 de 6 mm ficam 3 mm de rosca na torre
parede_larg    = 74;     // X: furos do Uno (±31,7) e margem para a cabeça do M3
placa_topo     = 3;      // Z onde começa a placa do Arduino (aresta de cima)
uno_margem     = 3;      // parede abaixo da aresta de baixo do Uno
folga_corpo    = 0.6;    // folga entre a parede e a face do corpo da pinça
m3_passagem    = 3.4;

/* [Placa da IMU (Grove 40 x 20 x 1,75), ficheiro Eagle da Seeed] */
imu_comp       = 40;
imu_larg       = 20;
imu_esp        = 1.75;
imu_saida      = 7.2;    // quanto a placa sai para lá da aresta da parede (a ficha)
imu_z          = 24.6;   // Z do centro da placa: entre as torres do Uno
imu_chip_dx    = 6.16;   // do chip à aresta curta do lado do chip
// A placa prende-se pela aresta: parafusos M3 x 10 por fora do contorno, com a
// cabeça (Ø 5,5) a apertar 1,2 mm da aresta da placa, e porca M3 do lado de
// dentro da parede. Posições a contar da aresta do lado do chip (X) e do
// centro (Z). Nas arestas compridas ficam nas zonas sem cobre do ficheiro
// Eagle (a 14,2 mm da aresta do lado do chip); o terceiro a meio da aresta do
// lado do chip serve de batente: encostar lá a placa.
m3_aperto      = 1.6;    // do centro do parafuso à aresta da placa (corpo Ø 3 + folga)
imu_furos      = [[14.2, imu_larg / 2 + m3_aperto], [14.2, -imu_larg / 2 - m3_aperto], [-m3_aperto, 0]];

/* [Reforços, cabo e marca] */
nervura_esp    = 3;
nervura_alt    = 38;     // até onde descem as nervuras (Z)
rasgo          = [2.2, 5];   // rasgos para abraçadeira do cabo USB
numero         = "1";

$fn = 48;

// Furos de fixação do Arduino Uno (mm, placa de 68,6 x 53,3 vista de cima,
// fichas à esquerda). A parede leva o padrão e o seu espelho: o Uno pode ir
// virado para qualquer dos lados.
uno = [[14.0, 2.5], [15.2, 50.8], [66.0, 7.6], [66.0, 35.6]];
uno_larg = 68.6;  uno_alt = 53.3;
parede_alt = placa_topo + uno_alt + uno_margem;   // Z onde acaba a parede
imu_x1 = parede_larg / 2 + imu_saida;             // X da aresta da placa do lado da ficha
imu_x0 = imu_x1 - imu_comp;                       // X da aresta do lado do chip

module furos_uno() {
    for (h = uno) for (s = [1, -1])
        translate([s * (h[0] - uno_larg / 2), 0, placa_topo + h[1]])
            rotate([-90, 0, 0]) children();
}

module base() {
    translate([-dedo_larg / 2, 0, 0]) cube([dedo_larg, dedo_prof, base_alt]);
    // rebordos que abraçam o carro (sobem acima da face do carro, Z < 0)
    for (s = [1, -1])
        translate([s > 0 ? carro_larg / 2 + folga_carro : -dedo_larg / 2, 0, -rebordo[1]])
            cube([dedo_larg / 2 - carro_larg / 2 - folga_carro, dedo_prof, rebordo[1] + 0.01]);
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
            translate([0, 0, -rebordo[1] - 1]) cylinder(d = m25_passagem, h = base_alt + rebordo[1] + 2);
            translate([0, 0, sob_cabeca]) cylinder(d = m25_cabeca, h = parede_alt);
        }
        // parafusos M3 das torres do Uno (cabeça do lado de dentro, à vista)
        furos_uno() translate([0, 0, lamina_y - 1]) cylinder(d = m3_passagem, h = parede_esp + 2);
        // parafusos M3 que apertam a placa da IMU pela aresta
        for (f = imu_furos)
            translate([imu_x0 + f[0], lamina_y - 1, imu_z + f[1]])
                rotate([-90, 0, 0]) cylinder(d = m3_passagem, h = parede_esp + 2);
        // rasgos para a abraçadeira do cabo
        for (s = [1, -1]) for (dz = [0, 9])
            translate([s * (parede_larg / 2 - 3) - rasgo[0] / 2, lamina_y - 1, parede_alt - 18 + dz])
                cube([rasgo[0], parede_esp + 2, rasgo[1]]);
        // número da peça, gravado 0,6 mm na face de baixo da base
        translate([0, dedo_prof / 2 - 2, base_alt - 0.6])
            linear_extrude(1) text(numero, size = 6, halign = "center", valign = "center",
                                   font = "Liberation Sans:style=Bold");
    }
}

// ---- contexto, só para ver (não entra no STL): pinça, o outro dedo, a IMU e o Uno
module contexto() {
    color("gainsboro", 0.5) translate([0, 0, -34]) cylinder(d = 77.5, h = 34, $fn = 96);
    color("dimgray") {                                   // o dedo que fica
        translate([-10, -15.3, 0]) cube([20, 15.3, 7]);
        translate([-10, -15.3, 0]) cube([20, 4.2, 23.5]);
    }
    y0 = lamina_y + parede_esp;
    color("seagreen") translate([imu_x0, y0, imu_z - imu_larg / 2]) cube([imu_comp, imu_esp, imu_larg]);
    color("white") translate([imu_x1 - 8.1, y0 + imu_esp, imu_z - 6.1]) cube([6.7, 8.25, 12.2]);   // ficha Grove
    color("black") translate([imu_x0 + imu_chip_dx - 1.5, y0 + imu_esp, imu_z - 1.25]) cube([3, 0.9, 2.5]);
    color("silver") for (f = imu_furos)
        translate([imu_x0 + f[0], y0 + imu_esp, imu_z + f[1]]) rotate([-90, 0, 0]) cylinder(d = 5.5, h = 3);
    color("goldenrod") furos_uno() translate([0, 0, y0]) cylinder(d = 5, h = 16);    // torres
    if (mostrar_uno) color("royalblue", 0.8) translate([-uno_larg / 2, y0 + 16, placa_topo])
        cube([uno_larg, 14, uno_alt]);                                               // Uno e fichas
}

mostrar_contexto = false;
mostrar_uno = true;
// vira-se o modelo para Z ficar para baixo no ecrã, como no robô
rotate([180, 0, 0]) {
    color("darkorange") suporte();
    if (mostrar_contexto) contexto();
}
