// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Constantes de campo do REBUILT 2026: layout oficial de AprilTags e
 * posições calculadas dos hubs de cada aliança.
 *
 * Fonte única de verdade — qualquer código do robô que precise saber "onde
 * fica o hub" deve consultar esta classe, em vez de guardar coordenadas
 * chumbadas manualmente em RobotContainer ou em qualquer subsystem.
 */
public final class FieldConstants {

    private FieldConstants() {
        // Classe utilitária: não deve ser instanciada.
    }

    // Se o seu evento usar o campo AndyMark em vez do Welded, troque para
    // AprilTagFields.k2026RebuiltAndyMark aqui.
    private static final AprilTagFieldLayout FIELD_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // IDs oficiais das AprilTags do HUB no manual do REBUILT 2026 (Seção 5.11),
    // já separados em dois grupos de 8 (um por hub) conforme confirmado no
    // diagrama oficial de referência de IDs do campo.
    private static final int[] HUB_GRUPO_A = {2, 3, 4, 5, 8, 9, 10, 11};
    private static final int[] HUB_GRUPO_B = {18, 19, 20, 21, 24, 25, 26, 27};

    /** Posição calculada do hub da aliança azul. Resolvida uma única vez, no carregamento da classe. */
    public static final Translation2d HUB_AZUL;

    /** Posição calculada do hub da aliança vermelha. Resolvida uma única vez, no carregamento da classe. */
    public static final Translation2d HUB_VERMELHO;

    static {
        double meioCampo = FIELD_LAYOUT.getFieldLength() / 2.0;

        Translation2d centroGrupoA = mediaDasTags(HUB_GRUPO_A);
        Translation2d centroGrupoB = mediaDasTags(HUB_GRUPO_B);

        // O AprilTagFieldLayout decide qual grupo é azul/vermelho pela posição X real
        // (origem do campo fica na parede da aliança azul) — os GRUPOS em si já vêm
        // fixos e confirmados visualmente, só a "cor" de cada grupo é dinâmica.
        if (centroGrupoA.getX() < meioCampo) {
            HUB_AZUL = centroGrupoA;
            HUB_VERMELHO = centroGrupoB;
        } else {
            HUB_AZUL = centroGrupoB;
            HUB_VERMELHO = centroGrupoA;
        }
    }

    private static Translation2d mediaDasTags(int[] ids) {
        List<Translation2d> pontos = new ArrayList<>();
        for (int id : ids) {
            Optional<Pose3d> pose = FIELD_LAYOUT.getTagPose(id);
            pose.ifPresent(p -> pontos.add(p.toPose2d().getTranslation()));
        }
        return mediaDasTranslacoes(pontos);
    }

    private static Translation2d mediaDasTranslacoes(List<Translation2d> pontos) {
        if (pontos.isEmpty()) {
            // Fallback de segurança: se por algum motivo nenhuma tag foi encontrada,
            // não queremos um NullPointerException travando o robô inteiro.
            return new Translation2d(0, 0);
        }
        double somaX = 0;
        double somaY = 0;
        for (Translation2d ponto : pontos) {
            somaX += ponto.getX();
            somaY += ponto.getY();
        }
        return new Translation2d(somaX / pontos.size(), somaY / pontos.size());
    }

    /** Retorna a posição do hub da aliança atual (a que o DriverStation reporta agora). */
    public static Translation2d getHubAtual() {
        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        return isRed ? HUB_VERMELHO : HUB_AZUL;
    }
}
