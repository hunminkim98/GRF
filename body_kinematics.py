import opensim
import numpy as np
import os
from load_file import load_sto, load_mot

def body_kinematics(
    model_path,
    motion_file,
    start_time,
    end_time,
    cutoff_freq
):
    """
    1) OpenSim AnalyzeTool + BodyKinematics 실행:
       _BodyKinematics_pos_global.sto,
       _BodyKinematics_vel_global.sto,
       _BodyKinematics_acc_global.sto 파일 생성
    2) 생성된 .sto 파일에서 (calcn_l, calcn_r, hand_l, hand_r) 위치, 속도, 가속도를 읽는다.
    3) MATLAB 코드와 동일하게 발 최소 높이 위치(range_pos_r, range_pos_l)와
       해당 시점(position_r, position_l)을 찾는다.
    
    Args:
        model_path (str): .osim 모델 경로
        motion_file (str): .mot IK 결과 파일 경로
        start_time (float): 분석 시작 시간
        end_time (float): 분석 종료 시간
        cutoff_freq (float): BodyKinematics에 적용할 로우패스 필터 주파수(Hz)

    Returns:
        dict: 분석 결과 (MATLAB과 동일한 구조)
              - 'time': 시간 배열
              - 'calcn_l', 'calcn_r', 'hand_l', 'hand_r' 각 { 'pos', 'vel', 'acc' }
              - 'position_l', 'position_r' (발 최소 높이 시점 인덱스)
    """
    # ------------------------------------------------------------------
    # 1) 모델 불러오기
    # ------------------------------------------------------------------
    model = opensim.Model(model_path)
    model.initSystem()  # 모델 초기화
    
    # ------------------------------------------------------------------
    # 2) AnalyzeTool 생성 및 설정
    # ------------------------------------------------------------------
    # 먼저 BodyKinematics 분석 도구 생성
    bk_tool = opensim.BodyKinematics()
    bk_tool.setName("BodyKinematics")  # 분석 도구 이름 설정
    bk_tool.setStartTime(start_time)
    bk_tool.setEndTime(end_time)
    bk_tool.setInDegrees(True)  # 각도 단위 설정
    
    # AnalysisSet 생성 및 BodyKinematics 추가
    analyses = opensim.AnalysisSet()
    analyses.adoptAndAppend(bk_tool)
    
    # AnalyzeTool 생성 (분석 도구가 포함된 Model로)
    model.addAnalysis(bk_tool)
    analysis_tool = opensim.AnalyzeTool(model)
    
    # 기타 설정
    analysis_tool.setModelFilename(model_path)
    analysis_tool.setCoordinatesFileName(motion_file)
    analysis_tool.setLoadModelAndInput(True)  # ERROR FIX 1
    
    analysis_tool.setStartTime(start_time)
    analysis_tool.setFinalTime(end_time)
    analysis_tool.setLowpassCutoffFrequency(cutoff_freq)
    analysis_tool.setSolveForEquilibrium(False)

    # 결과 폴더
    results_dir = os.path.join(os.path.dirname(motion_file), "BK")
    os.makedirs(results_dir, exist_ok=True)
    analysis_tool.setResultsDir(results_dir)

    # 설정 파일 (Setup_BK.xml)로 출력
    setup_file = os.path.join(os.path.dirname(motion_file), "Setup_BK.xml")
    analysis_tool.printToXML(setup_file)

    # ------------------------------------------------------------------
    # 4) 분석 실행 → _BodyKinematics_*_global.sto 파일들 생성
    # ------------------------------------------------------------------
    analysis_tool.run()

    # ------------------------------------------------------------------
    # 5) 생성된 .sto 파일 읽기 (pos, vel, acc)
    # ------------------------------------------------------------------
    pos_file = os.path.join(results_dir, "_BodyKinematics_pos_global.sto")
    vel_file = os.path.join(results_dir, "_BodyKinematics_vel_global.sto")
    acc_file = os.path.join(results_dir, "_BodyKinematics_acc_global.sto")
    print(f"{pos_file} loaded")
    print(f"{vel_file} loaded")
    print(f"{acc_file} loaded")

    BodyPos, Head_BK = load_sto(pos_file)
    BodyVel, Head_BK = load_sto(vel_file)
    BodyAcc, Head_BK = load_sto(acc_file)
    print(f"{pos_file} loaded")
    print(f"{vel_file} loaded")
    print(f"{acc_file} loaded")

    # 시간 벡터
    time_array = BodyPos[:, 0]  # 첫 열 = time
    n_rows = len(time_array)

    # ------------------------------------------------------------------
    # 6) 필요한 컬럼 인덱스 (calcn_l, calcn_r, hand_l, hand_r)
    # ------------------------------------------------------------------
    def get_indices(headers, prefix):
        x_idx = headers.index(f"{prefix}_X")
        y_idx = headers.index(f"{prefix}_Y")
        z_idx = headers.index(f"{prefix}_Z")
        return x_idx, y_idx, z_idx

    calcn_l_x_idx, calcn_l_y_idx, calcn_l_z_idx = get_indices(Head_BK, "calcn_l")
    calcn_r_x_idx, calcn_r_y_idx, calcn_r_z_idx = get_indices(Head_BK, "calcn_r")
    hand_l_x_idx, hand_l_y_idx, hand_l_z_idx    = get_indices(Head_BK, "hand_l")
    hand_r_x_idx, hand_r_y_idx, hand_r_z_idx    = get_indices(Head_BK, "hand_r")

    # ------------------------------------------------------------------
    # 7) pos, vel, acc 배열로 정리
    # ------------------------------------------------------------------
    # 위치
    calcn_l_pos = np.zeros((n_rows, 3))
    calcn_r_pos = np.zeros((n_rows, 3))
    hand_l_pos  = np.zeros((n_rows, 3))
    hand_r_pos  = np.zeros((n_rows, 3))

    calcn_l_pos[:, 0] = BodyPos[:, calcn_l_x_idx]
    calcn_l_pos[:, 1] = BodyPos[:, calcn_l_y_idx]
    calcn_l_pos[:, 2] = BodyPos[:, calcn_l_z_idx]

    calcn_r_pos[:, 0] = BodyPos[:, calcn_r_x_idx]
    calcn_r_pos[:, 1] = BodyPos[:, calcn_r_y_idx]
    calcn_r_pos[:, 2] = BodyPos[:, calcn_r_z_idx]

    hand_l_pos[:, 0] = BodyPos[:, hand_l_x_idx]
    hand_l_pos[:, 1] = BodyPos[:, hand_l_y_idx]
    hand_l_pos[:, 2] = BodyPos[:, hand_l_z_idx]

    hand_r_pos[:, 0] = BodyPos[:, hand_r_x_idx]
    hand_r_pos[:, 1] = BodyPos[:, hand_r_y_idx]
    hand_r_pos[:, 2] = BodyPos[:, hand_r_z_idx]

    # 속도
    calcn_l_vel = np.zeros((n_rows, 3))
    calcn_r_vel = np.zeros((n_rows, 3))

    calcn_l_vel[:, 0] = BodyVel[:, calcn_l_x_idx]
    calcn_l_vel[:, 1] = BodyVel[:, calcn_l_y_idx]
    calcn_l_vel[:, 2] = BodyVel[:, calcn_l_z_idx]

    calcn_r_vel[:, 0] = BodyVel[:, calcn_r_x_idx]
    calcn_r_vel[:, 1] = BodyVel[:, calcn_r_y_idx]
    calcn_r_vel[:, 2] = BodyVel[:, calcn_r_z_idx]

    # 가속도
    calcn_l_acc = np.zeros((n_rows, 3))
    calcn_r_acc = np.zeros((n_rows, 3))

    calcn_l_acc[:, 0] = BodyAcc[:, calcn_l_x_idx]
    calcn_l_acc[:, 1] = BodyAcc[:, calcn_l_y_idx]
    calcn_l_acc[:, 2] = BodyAcc[:, calcn_l_z_idx]

    calcn_r_acc[:, 0] = BodyAcc[:, calcn_r_x_idx]
    calcn_r_acc[:, 1] = BodyAcc[:, calcn_r_y_idx]
    calcn_r_acc[:, 2] = BodyAcc[:, calcn_r_z_idx]

    # ------------------------------------------------------------------
    # 8) 발 높이(y좌표) 최소값 주변 시점 찾기
    # ------------------------------------------------------------------
    p_calcn_l_y = calcn_l_pos[:, 1]
    p_calcn_r_y = calcn_r_pos[:, 1]

    min_calcn_l_y = np.min(p_calcn_l_y)
    min_calcn_r_y = np.min(p_calcn_r_y)

    range_pos_l = p_calcn_l_y < (min_calcn_l_y + min_calcn_l_y * 1.0/1000.0)
    range_pos_r = p_calcn_r_y < (min_calcn_r_y + min_calcn_r_y * 1.0/1000.0)

    min_p_calcn_l_y_idx = np.where(range_pos_l)[0]
    min_p_calcn_r_y_idx = np.where(range_pos_r)[0]

    a_calcn_l_y = calcn_l_acc[:, 1]
    a_calcn_r_y = calcn_r_acc[:, 1]

    # MATLAB: [val_pos_r, pos_r] = min(abs(a_calcn_r_y(min_p_calcn_r_y)))
    #         position_r = min_p_calcn_r_y(pos_r)
    if len(min_p_calcn_r_y_idx) > 0:
        abs_vals_r = np.abs(a_calcn_r_y[min_p_calcn_r_y_idx])
        pos_r      = np.argmin(abs_vals_r)
        position_r = min_p_calcn_r_y_idx[pos_r]
    else:
        position_r = None

    if len(min_p_calcn_l_y_idx) > 0:
        abs_vals_l = np.abs(a_calcn_l_y[min_p_calcn_l_y_idx])
        pos_l      = np.argmin(abs_vals_l)
        position_l = min_p_calcn_l_y_idx[pos_l]
    else:
        position_l = None

    # ------------------------------------------------------------------
    # 9) 결과 반환
    # ------------------------------------------------------------------
    return {
        "time": time_array,
        "calcn_l": {
            "pos": calcn_l_pos,
            "vel": calcn_l_vel,
            "acc": calcn_l_acc
        },
        "calcn_r": {
            "pos": calcn_r_pos,
            "vel": calcn_r_vel,
            "acc": calcn_r_acc
        },
        "hand_l": {
            "pos": hand_l_pos
        },
        "hand_r": {
            "pos": hand_r_pos
        },
        "position_l": position_l,
        "position_r": position_r
    }