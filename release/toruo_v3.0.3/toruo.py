﻿"""
撮る夫くん Ver. 3.0.3 Beta
2019.7.03
VRMNXでグローバルカメラの高機能な操作システムを提供します
v.3.0.0 Beta    NX用初版
v.3.0.1 Beta    被写界深度に対応
v.3.0.2 Beta    子分の保存に対応
v.3.0.3 Beta    追尾・オートフォーカス・プログラムオート
"""
VERSION = '3.0.3'

try:
    import vrmapi
except ModuleNotFoundError:
    print('VRMAPIが見つかりません。VRMNXシステムでしか動作しません。')
    raise ModuleNotFoundError
from math import sin,cos,tan,sqrt, pi, pow
# from threading import Thread, Timer
# from time import sleep
from random import triangular
import json

DEBUG = True

LAYOUT = vrmapi.LAYOUT()
NXSYS = LAYOUT.SYSTEM()
IMGUI = vrmapi.ImGui()
# LAYOUTにFrameイベントを登録
LAYOUT.SetEventFrame()

_PARENT = None     # 親オブジェクト
#_gcam = []       # 地上カメラのリスト
_toruos = []     # 撮る夫くんたちのリスト
_childid = [0]
_systime = 0.0 # 前フレームの時刻を記録
_shakemode = [False] # Trueで手ブレON
_guidisp = 0       # TrueでGUI操作盤を表示
_shake_vt = 0.0 # 手ブレの累積量
_shake_hr = 0.0
_shake_dvt = 0.0 # 手ブレの差分
_shake_dhr = 0.0
_shake_evid = None
_aemode = [False]   # プログラムオート
_aeparam = {'blurfin':[90.0], 'ftg':[5.0/12], 'f10':[25.0]}

# DOF(被写界深度)がらみのパラメタ
_fov = [45.0]
_depth = [0.25]         # 合焦中心距離の逆数の４乗根
_fnum = [50.0]          # F値
_blur = [1.0]
_delta = [0.0008, 0.08]

# 設定値（公開プロパティ）
dFOV = 10.0
dRot = 0.5
dMov = 25.0
shake_factor = 0.1
shake_freq = 4.0

# 追尾モード
_trainlist = {'obj':[], 'id':[], 'name':[]}
_tracking_mode = [False]
_tracking_car = None
_tracking_trainid = [0]
_tracking_trnlen = 0
_tracking_carnum = [0]
_tracking_dist = [256.0]
_tracking_relative = {'x':[0.0], 'y':[0.0], 'z':[0.0]}
_tracking_af = [False]


def load_config(filename='toruo.json'):
    global _toruos
    try:
        with open(filename) as js:
            _toruos = json.load(js)
    except FileNotFoundError:
        pass

def save_config(filename='toruo.json'):
    with open(filename, 'w') as js:
        json.dump(_toruos, js, indent=4)

def jump_toruo(id=0):
    global _fov
    global _depth
    global _fnum
    global _blur
    NXSYS.SetGlobalCameraPos(_toruos[id]['pos'])
    _fov[0] = _toruos[id]['fov']
    _depth[0] = _toruos[id]['depth']
    _fnum[0] = _toruos[id]['fnum']
    _blur[0] = _toruos[id]['blur']
    _focus()

def save_toruo():
    d = dict()
    d['pos'] = NXSYS.GetGlobalCameraPos()
    d['fov'] = _fov[0]
    d['depth'] = _depth[0]
    d['fnum'] = _fnum[0]
    d['blur'] = _blur[0]
    _toruos.append(d)
    save_config()


def setfactor(rotate=0.5, fov=10.0, move=25.0):
    global dRot
    global dFOV
    global dMov
    dRot = rotate
    dFOV = fov
    dMov = move

def setshakemode(mode=False):
    """ mode=Trueで手ブレON """
    global _shakemode
    global _shake_hr
    global _shake_vt
    _shake_vt = 0.0 # 手ブレ量をリセット
    _shake_hr = 0.0
    _shakemode[0] = mode
    _update_shake()

def _update_shake():
    global _shake_dhr
    global _shake_dvt
    global _shake_evid
    _shake_dhr = triangular(-1*shake_factor, shake_factor)
    _shake_dvt = triangular(-1*shake_factor, shake_factor)
    if _shakemode[0]:
        _shake_evid = _PARENT.SetEventAfter(triangular(0.0, 1.0/shake_freq))
            
def activate(obj, ev, param):
    """ 撮る夫くんをアクティベートするメソッド
    イベントハンドラの直下に書いてください（ifの中に入れない） """
    if ev == 'init':
        global _PARENT
        _PARENT = obj
        load_config()
        obj.SetEventKeyDown('P')
        _refresh_trainlist()
        vrmapi.LOG('撮る夫くん(Ver.{}) stand by.'.format(VERSION))
        return
    elif param['eventid'] == _shake_evid:
        # (Afterイベント)
        _update_shake()
        return
    elif ev == 'keydown':
        if param['keycode'] == 'P':
            # GUI表示をON/OFF
            global _guidisp
            _guidisp = (_guidisp+1)%2
        return

    if ev != 'frame':
        return

    if _guidisp:
        _dispgui()
    if not LAYOUT.IsViewGlobal():
        return
    ftime = _updateframetime(param['eventtime'])
    campos = NXSYS.GetGlobalCameraPos()
    global _shake_hr
    global _shake_vt

    # ズームイン
    stat = NXSYS.GetKeyStat('E') + NXSYS.GetKeyStat('W') + NXSYS.GetKeyStat('R') + 3.0
    if stat > 0.0:
        _zoom(-1, ftime)

    # ズームアウト
    stat = NXSYS.GetKeyStat('S') + NXSYS.GetKeyStat('D') + NXSYS.GetKeyStat('F') + 3.0
    if stat > 0.0:
        _zoom(1, ftime)

    # 左回転
    stat = NXSYS.GetKeyStat('W') + NXSYS.GetKeyStat('S') + 2.0
    if stat > 0.0:
        _rotate(campos, -1, ftime)

    # 右回転
    stat = NXSYS.GetKeyStat('R') + NXSYS.GetKeyStat('F') + 2.0
    if stat > 0.0:
        _rotate(campos, 1, ftime)

    # 追尾処理
    istracking = False
    if _tracking_mode[0] and _tracking_car:
        rel = [_tracking_relative['x'][0], _tracking_relative['y'][0], _tracking_relative['z'][0]]
        rel = vectrans(xrot_matrix3d(_tracking_car.GetRotateX()), rel)
        rel = vectrans(yrot_matrix3d(_tracking_car.GetRotateY()), rel)
        rel = vectrans(zrot_matrix3d(-1*_tracking_car.GetRotateZ()), rel)
        tgtpos = vecadd(_tracking_car.GetPosition(), rel)
        dist = vecdistance(campos[0:3], tgtpos)
        if dist < _tracking_dist[0]:
            istracking = True
            campos[3:6] = tgtpos
            if _tracking_af[0]:
                global _depth
                #global _blur
                # Auto Focus (Program Exposure)
                _depth[0] = pow(dist, -0.25)
                #_blur[0] = 2*(135.0-_fov[0])/125.0
                _focus()

    # ブレ計算
    if _shakemode[0]:
        if istracking:
            _shake_hr += _shake_dhr * dRot * ftime
            _shake_vt += _shake_dvt * dRot * ftime
            _rotate(campos, _shake_hr, 1.0)
            _rotatevt(campos, _shake_vt, 1.0)
        else:
            _rotate(campos, _shake_dhr, ftime)
            _rotatevt(campos, _shake_dvt, ftime)
            
    NXSYS.SetGlobalCameraPos(campos)

def _updateframetime(time_now):
    global _systime
    ftime = time_now - _systime
    _systime = time_now # フレームの時刻を記録
    return ftime

def _zoom(sgn, ftime):
    """ 
    sgn -1でズームイン，+1でズームアウト
    ftime フレーム描画時間
    """
    global _fov
    _fov[0] = NXSYS.GetGlobalCameraFOV() + sgn * dFOV * ftime
    NXSYS.SetGlobalCameraFOV(_fov[0])
    _focus()

def _rotate(campos, sgn, ftime):
    """ 水平方向見回し
    sgn -1で左，+1で右回り
    """
    # list campos はMutableなので返り値を取らなくていい
    dx = campos[3] - campos[0]
    dz = campos[5] - campos[2]
    dtheta = sgn * dRot * ftime
    cos_ = cos(dtheta)
    sin_ = sin(dtheta)
    campos[3] = campos[0] + cos_*dx - sin_*dz
    campos[5] = campos[2] + sin_*dx + cos_*dz

def _rotatevt(campos, sgn, ftime):
    """ 垂直見回し """
    x = sqrt((campos[3]-campos[0])**2 + (campos[5]-campos[2])**2)
    y = campos[4]-campos[1]
    alpha = tan(sgn * dRot * ftime)
    campos[4] = (y + x*alpha)/(1 - y/x*alpha) + campos[1] # tan加法定理の変形

def _focus():
    global _blur
    global _fnum
    NXSYS.SetGlobalCameraFOV(_fov[0])
    if _aemode[0]:
        _blur[0] = 2*(_aeparam['blurfin'][0]-_fov[0])/(_aeparam['blurfin'][0]-10.0)
        _fnum[0] = _aeparam['ftg'][0]*(_fov[0]-10.0)+_aeparam['f10'][0]
    d1 = _depth[0]**4
    d2 = _fnum[0] * tan(_fov[0]*pi/180.0)**2 / 100.0
    NXSYS.SetFocusParam(d1+_delta[0]*d2,d1-_delta[0]*d2,d1+_delta[1]*d2,d1-_delta[1]*d2, _blur[0])

def _refresh_trainlist():
    """ 編成リストを更新（分割・併合後には実行したい） """
    global _trainlist
    _trainlist['obj'] = []
    LAYOUT.ListTrain(_trainlist['obj'])
    _trainlist['id'] = [t.GetID() for t in _trainlist['obj']]
    _trainlist['name'] = [t.GetNAME() for t in _trainlist['obj']]

def _dispgui():
    global _shakemode
    global _fov
    global _depth
    global _fnum
    global _trainlist
    global _tracking_mode
    global _tracking_car
    global _tracking_trainid
    global _tracking_trnlen
    global _tracking_carnum
    global _tracking_relative
    global _tracking_af
    global _aemode
    global _aeparam
    IMGUI.Begin("ToruoWin", "撮る夫くん")
    action = False
    action += IMGUI.SliderFloat("zoom", "FOV(ズーム角度)", _fov, 10.0, 135.0)
    action += IMGUI.SliderFloat("focus", "合焦中心", _depth, 0.1, 1.0)
    #if not _aemode[0]:
    action += IMGUI.SliderFloat("fnum", "絞り", _fnum, 1.0, 50.0)
    action += IMGUI.SliderFloat("blur", "ぼけの強さ", _blur, 0.0, 2.0)
    if action:
        _focus()
    IMGUI.Separator()
    if IMGUI.Checkbox("program_ae", "プログラムAE", _aemode):
        _focus()
    IMGUI.SameLine()
    if IMGUI.Checkbox("shake", "手ブレモード", _shakemode):
        setshakemode(_shakemode[0])
    if IMGUI.TreeNode("childbtn",  "保存済み撮る夫くん"):
        if _toruos:
            for i,d in enumerate(_toruos):
                if IMGUI.RadioButton("toruo{}".format(i), "撮る夫くん {}".format(i), _childid, i):
                    jump_toruo(i)
                    # vrmapi.LOG('撮る夫くん {}'.format(i))
        else:
            IMGUI.Text("撮る夫くんは保存されていません")
        if IMGUI.Button("addtoruo", "現在視点を保存"):
            save_toruo()
        IMGUI.SameLine()
        if IMGUI.Button("deltoruo", "現在の撮る夫くんを削除"):
            del _toruos[_childid[0]]
            save_config()
        IMGUI.TreePop()

    if IMGUI.TreeNode("target", "追尾モード"):
        IMGUI.Checkbox("trackmode", "追尾モード", _tracking_mode)
        IMGUI.SameLine()
        IMGUI.Checkbox("trackaf", "オートフォーカス", _tracking_af)
        if IMGUI.TreeNode("targettrn", "対象の編成"):
            if IMGUI.Button("trnlist", "編成リストを更新"):
                _refresh_trainlist()
            if IMGUI.RadioButton("notrack", "追尾なし", _tracking_trainid, 0):
                _tracking_car = None
            for i, tid in enumerate(_trainlist['id']):
                if IMGUI.RadioButton("trn{}".format(tid), _trainlist['name'][i], _tracking_trainid, tid):
                    _tracking_carnum[0] = 1
                    _tracking_car = LAYOUT.GetTrain(_tracking_trainid[0]).GetCar(_tracking_carnum[0]-1)
                    _tracking_trnlen = _trainlist['obj'][i].GetNumberOfCars()
                    vrmapi.LOG("[撮る夫くん]追尾対象変更 {}".format(_trainlist['name'][i]))
            if _tracking_trainid[0]:
                if IMGUI.SliderInt("carno", "号車番号", _tracking_carnum, 1, _tracking_trnlen):
                    _tracking_car = LAYOUT.GetTrain(_tracking_trainid[0]).GetCar(_tracking_carnum[0]-1)
                IMGUI.Text("車体長: {0:.1f}mm".format(vecdistance(_tracking_car.GetLinkPosition(0), _tracking_car.GetLinkPosition(1))))
            IMGUI.SliderFloat("relx", "相対X", _tracking_relative['x'], -150.0, 150.0)
            IMGUI.SliderFloat("rely", "相対Y", _tracking_relative['y'], -50.0, 50.0)
            IMGUI.SliderFloat("relz", "相対Z", _tracking_relative['z'], -50.0, 50.0)
            IMGUI.TreePop()
        IMGUI.SliderFloat("trdist", "追尾距離", _tracking_dist, 100.0, 2500.0)
        IMGUI.Text(str(_tracking_car))
        IMGUI.TreePop()
    if IMGUI.TreeNode("details", "詳細設定"):
        IMGUI.Text("AE詳細設定")
        IMGUI.SliderFloat('ae1', 'ぼけ限界FOV', _aeparam['blurfin'], 35.0,135.0)
        IMGUI.SliderFloat('ae2', 'F増加率', _aeparam['ftg'], 0.1,1.0)
        IMGUI.SliderFloat('ae3', 'FOV10でのF', _aeparam['f10'], 1.0, 50.0)
        IMGUI.TreePop()
    IMGUI.Separator()
    if IMGUI.Button("closer", "メニューを閉じる"):
        global _guidisp
        _guidisp = 0
    IMGUI.End()

def vecadd(vec1, vec2):
    return [a+b for a,b in zip(vec1, vec2)]

def vecdistance(vec1, vec2):
    # ベクトル間の距離（ユークリッド距離）
    sq = 0.0
    for a,b in zip(vec1, vec2):
        sq += (a-b)**2
    return sqrt(sq)

def veclen(vec):
    # ベクトルの長さ
    sq = 0.0
    for a in vec:
        sq += a**2
    return sqrt(sq)

def vecdot(vec1, vec2):
    # 内積（スカラー）を返す
    p = 0.0
    for i, j in zip(vec1, vec2):
        p += i*j
    return p

def vectrans(matrixA, vecx):
    # 線形変換 Ax
    vecy = []
    for k in range(len(vecx)):
        vecy.append(vecdot(matrixA[k], vecx))
    return vecy

def turnmatrix(matrixA):
    # 行列の転置
    m = len(matrixA)
    n = len(matrixA[0])
    return [[matrixA[i][j] for i in range(m)] for j in range(n)]

def matrixproduct(matrixA, matrixB):
    # 行列の積
    Bt = turnmatrix(matrixB)
    return [vectrans(Bt, ai) for ai in matrixA]

def xrot_matrix3d(degree):
    theta = degree * pi / 180.0
    return [[1.0, 0.0, 0.0], [0.0, cos(theta), sin(theta)], [0.0, -1*sin(theta), cos(theta)]]

def yrot_matrix3d(degree):
    theta = degree * pi / 180.0
    return [[cos(theta), 0.0, -1*sin(theta)], [0.0, 1.0, 0.0], [sin(theta), 0.0, cos(theta)]]
    
def zrot_matrix3d(degree):
    theta = degree * pi / 180.0
    return [[cos(theta), sin(theta), 0.0], [-1*sin(theta), cos(theta), 0.0], [0.0, 0.0, 1.0]]

if __name__== "__main__":
    print('toruoはVRMNXシステムでのみ有効なモジュールです')
    input("Press any key to close...")