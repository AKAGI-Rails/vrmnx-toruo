# -*- coding: utf-8 -*-
"""撮る夫くん - VRMNX用グローバルカメラ拡張機能

VRMNX用Python拡張の **撮る夫くん** は，VRMNXビュワーのフライスルーカメラの機能をアップグレードします。
ImGUIの操作パネルで，FOVや被写界深度の設定を直感的に行うことができます。

撮る夫くんには以下のような機能があります。

- FOVなどのGUI操作
- FOV, F値と対象物までの距離から，一眼レフカメラの機構をシミュレートした被写界深度制御
- 手ブレ風エフェクト
- 車両追尾
- 視点保存

Example:
    撮る夫くんを有効にするには，レイアウトのイベントハンドラの冒頭に
    `toruo.activate()` を記述します::
    
        import vrmnx
        import toruo
        
        def vrmevent(obj,ev,param):
            toruo.activate(obj,ev,param)
            if ev == 'init':
                pass
    
# イベントのuserIDの予約領域

撮る夫くんでは，VRMNXのイベントuserIDで以下の領域を予約します。::

    1060000 - 1069999

撮る夫くんが内部で使用するイベントはこの領域内でuserIDを指定しています。
"""

__all__ = ['DEBUG', 'dFOV', 'dRot', 'dMov', 'shake_factor', 'shake_freq',
           'activate', 'set_toruo', 'jump_toruo', 'setfactor', 'setshakemode', 'set_gcdist',]
__version__ = '3.2.0'
__author__ = "AKAGI"

try:
    import vrmapi
    LOG = vrmapi.LOG
except ModuleNotFoundError:
    print('VRMAPIが見つかりません。VRMNXシステムでしか動作しません。')
    raise
from math import sin,cos,tan,sqrt, pi, pow, ceil,floor
from random import triangular
import json
import os.path

DEBUG = True

LAYOUT = vrmapi.LAYOUT()
NXSYS = vrmapi.SYSTEM()
IMGUI = vrmapi.ImGui()
DIRECTORY, BASE = os.path.split(NXSYS.GetLayoutPath())
#LOG(BASE)

# フレームイベントの設定は activate イベントハンドラでinitのタイミングに移動

_PARENT = None  # 親オブジェクト
#_gcam = []     # 地上カメラのリスト
_toruos = []    # 撮る夫くんたちのリスト
_childid = [0]
_systime = 0.0  # 前フレームの時刻を記録
_shakemode = [False] # Trueで手ブレON
_guidisp = True      # TrueでGUI操作盤を表示
_shake_vt = 0.0      # 手ブレの累積量
_shake_hr = 0.0
_shake_dvt = 0.0     # 手ブレの差分
_shake_dhr = 0.0
_shake_evid = None
_aemode = [False]    # プログラムオート
_aeparam = {'blurfin':[90.0], 'ftg':[5.0/12], 'f10':[25.0]}
_sunpos = [[0], [45]] # 太陽位置(緯度,経度)
_gcdist = [256.0]    # グローバルカメラのfrom-to距離設定値（リアルタイムではない）

# DOF(被写界深度)がらみのパラメタ
_fov = [45.0]
_depth = [0.25]          # 合焦中心距離までの逆数の4乗根
_fnum = [50.0]           # F値
_blur = [1.0]            # ぼけの強さ
_delta = [0.0008, 0.08]  # 錯乱円径

# 設定値（公開プロパティ）
dFOV = 10.0         #: FOV操作感度
dRot = 0.5          #: 左右回転感度
dMov = 25.0         #: 移動量感度
shake_factor = 0.1  #: 手ブレ量
shake_freq = 4.0    #: 手ブレ周波数

# 追尾モード
_trainlist = {'obj':[], 'id':[], 'name':[]}
_tracking_mode = [False]
_fuzzytrack = [False]
_tracking_car = None
_tracking_trainid = [0]
_tracking_trnlen = 0
_tracking_carnum = [0]
_tracking_dist = [256.0]
_tracking_relative = {'x':[0.0], 'y':[0.0], 'z':[0.0]}
_tracking_af = [False]

# Event UserID
EVUID_TORUOFRAME = 1060000
EVUID_TORUOSWITCH = 1060001
EVUID_TORUOSHAKE = 1060101

def activate(obj, ev, param):
    """撮る夫くんを有効にするコマンド。
    
    レイアウトオブジェクトのイベントハンドラの先頭に書いてください。（ifの中には入れない。）::
    
        # LAYOUT
        import vrmapi
        import toruo
        
        def vrmevent(obj,ev,param):
            toruo.activate(obj,ev,param)
            if ev == 'init':
                pass # 以下省略
            
    Args:
        obj:    イベントハンドラが受け取るobj
        ev:     イベントハンドラが受け取るev
        param:  イベントハンドラが受け取るparam

    撮る夫くんの関係イベントはすべてこの関数の中で処理されます。
    `vrmevent` が受け取ったパラメータをそのまま引き渡してください。
    """
    if ev == 'init':
        global _PARENT
        _PARENT = obj
        obj.SetEventFrame(EVUID_TORUOFRAME)
        obj.SetEventKeyDown('P', EVUID_TORUOSWITCH)
        _load_config()
        _refresh_trainlist()
        vrmapi.LOG('撮る夫くん(Ver.{}) stand by. {}'.format(__version__, DIRECTORY))
        return
    elif param['eventid'] == _shake_evid:
        # (Afterイベント)
        _update_shake()
        return
    elif ev == 'keydown':
        if param['keycode'] == 'P':
            # GUI表示のON/OFFを切替
            global _guidisp
            _guidisp = not _guidisp
        return

    #if not (ev == 'frame' and param['eventUID'] == EVUID_TORUOFRAME):
    if ev != 'frame':
        # 撮る夫くんのフレームイベント以外ではreturn
        return

    if _guidisp:
        _dispgui()
    if not LAYOUT.IsViewGlobal():
        return
    ftime = _updateframetime(param['eventtime'])
    campos = NXSYS.GetGlobalCameraPos()

    # キー操作の処理
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
    istracking = False  # 初期化

    if _tracking_mode[0] and _tracking_car:
        if _fuzzytrack[0]:
            tgtpos = _tracktargetpos_fuzzy(_tracking_trainid[0], _tracking_carnum[0]-1)
        else:
            tgtpos = _getcarworldpos(car=_tracking_car)
        vrmapi.LOG(str(tgtpos))
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


def jump_toruo(id=0):
    """保存済みの撮る夫くん座標にジャンプ
    
    保存済みの撮る夫くんを``id``で呼び出し，その座標にジャンプします。
    
    Arg:
        id (int): 撮る夫くんの保存番号
        
    Note:
        保存された撮る夫くんの内容は，レイアウトと同じディレクトリの toruo.json に記録されています。
        リスト型で読み出されるので，先頭のデータがid=0になります。

    この関数は，実行時に撮る夫くん座標に即時反映されます。よって，適切なイベントハンドラの中で呼び出す必要があります。
    """
    global _fov
    global _depth
    global _fnum
    global _blur
    global _aemode
    global _aeparam
    d = _toruos[id]
    NXSYS.SetGlobalCameraPos(d['pos'])
    _fov[0] = d['fov']
    _depth[0] = d['depth']
    _fnum[0] = d['fnum']
    _blur[0] = d['blur']
    try:
        _aemode[0] = d['aemode']
        _aeparam = d['aeparam']
    except KeyError:
        pass
    _focus()


def set_toruo(fov=None, depth=None, fnum=None, blur=None, aemode=None):
    """撮る夫くんの状態を直接指定する。
    
    撮る夫くんの撮影設定パラメータを直接指定できます。
    入力パラメータはどれも省略可です。必要な項目だけキーワード引数で指定してください。

    Parameters:
        fov (float): FOV（ズーム角度）
        depth (float): 合焦中心位置までの距離の逆数の4乗根
        fnum (float): 絞り（F値）
        blur (float): ぼけの強さ
        aemode (bool): プログラムAE
    
    この関数は，実行時に即時反映されます。よって，適切なイベントハンドラの中で呼び出す必要があります。

    たとえば，`init`イベントで実行すると初期状態の撮る夫くんの設定ができます。::

        # LAYOUT
        import vrmapi
        import toruo
        
        def vrmevent(obj,ev,param):
            toruo.activate(obj,ev,param)
            if ev == 'init':
                toruo.set_toruo(depth=512**(-1/4), fnum=9.0)

    カメラ座標は `vrmapi.SYSTEM().SetGlobalCameraPos()` によってください。
        
    """
    global _fov
    global _depth
    global _fnum
    global _blur
    global _aemode
    global _aeparam
    if fov is not None:
        _fov[0] = fov
    if depth is not None:
        _depth[0] = depth
    if fnum is not None:
        _fnum[0] = fnum
    if blur is not None:
        _blur[0] = blur
    if aemode is not None:
        _aemode[0] = aemode
    _focus()


def setfactor(rotate=0.5, fov=10.0, move=25.0):
    """キー操作と手ブレの感度パラメータを設定
    
    v.3.0.7ではmoveは不使用です。 
    
    Args:
        rotate (float): 画角回転の感度 (default=0.5)
        fov (float): ズーム感度 (default=10.0)
    
    """
    global dRot
    global dFOV
    global dMov
    dRot = rotate
    dFOV = fov
    dMov = move


def setshakemode(mode=False):
    """手ブレモードを設定
    
    Args:
        mode (Bool): Trueで手ブレON
    """
    global _shakemode
    global _shake_hr
    global _shake_vt
    _shake_vt = 0.0 # 手ブレ量をリセット
    _shake_hr = 0.0
    _shakemode[0] = mode
    _update_shake()


def set_gcdist(dist=256.0):
    """グローバルカメラのfrom-toの距離を再設定する。
    
    現在の視線の向きをキープして，グローバルカメラのat座標を
    from座標からの指定距離に再設定する。

    Parameter:
        dist: from-toの距離(mm)
    """
    pos = NXSYS.GetGlobalCameraPos()
    pos_from = pos[0:3]
    pos_at = pos[3:6]
    vec_eye = vecadd(pos_at, vecscale(-1, pos_from))  # at - from

    distnow = veclen(vec_eye)
    new_eye = list(map(lambda x: dist/distnow*x, vec_eye))
    pos[3:6] = vecadd(pos_from , new_eye)
    NXSYS.SetGlobalCameraPos(pos)


def _update_shake():
    global _shake_dhr
    global _shake_dvt
    global _shake_evid
    _shake_dhr = triangular(-1*shake_factor, shake_factor)
    _shake_dvt = triangular(-1*shake_factor, shake_factor)
    if _shakemode[0]:
        _shake_evid = _PARENT.SetEventAfter(triangular(0.0, 1.0/shake_freq), EVUID_TORUOSHAKE)
            

def _save_toruo():
    """現在視点を保存。"""
    d = dict()
    d['pos'] = NXSYS.GetGlobalCameraPos()
    d['fov'] = _fov[0]
    d['depth'] = _depth[0]
    d['fnum'] = _fnum[0]
    d['blur'] = _blur[0]
    d['aemode'] = _aemode[0]
    d['aeparam'] = _aeparam
    _toruos.append(d)
    _save_config()


def _load_config(filename=os.path.join(DIRECTORY, 'toruo.json')):
    """保存済み撮る夫くんを読み込み
    
    レイアウトと同じディレクトリの toruo.json を探し，存在すればデータを読み込みます。
    """
    global _toruos
    try:
        with open(filename) as js:
            _toruos = json.load(js)
    except FileNotFoundError:
        pass


def _save_config(filename=os.path.join(DIRECTORY, 'toruo.json')):
    """撮る夫くん保存状況をファイルに書き出し"""
    with open(filename, 'w') as js:
        json.dump(_toruos, js, indent=4)


def _getcarworldpos(trainid=None, carnum=None, car=None):
    """指定車両におけるglobalの_tracking_relativeにある相対座標をレイアウト上の絶対座標にして返す
    
    ``trainid``と``carnum``の両方を指定するか，直接``car``を指定してください。
    
    Args:
        trainid (int, optional): 編成オブジェクトのID.
        carnum (int, optional): 号車番号（先頭=0号車）
        car (:obj:`vrmapi.VRMCar`): VRM車両オブジェクト
    """
    if not car:
        car = LAYOUT.GetTrain(trainid).GetCar(carnum)
    rel = [_tracking_relative['x'][0], _tracking_relative['y'][0], _tracking_relative['z'][0]]
    rel = vectrans(xrot_matrix3d(car.GetRotateX()), rel)
    rel = vectrans(yrot_matrix3d(car.GetRotateY()), rel)
    rel = vectrans(zrot_matrix3d(-1*car.GetRotateZ()), rel)
    return vecadd(car.GetPosition(), rel)


def _tracktargetpos_fuzzy(trainid, carnum):
    """ファジィ号車番号で編成の相対座標を絶対座標に変換。
    
    carnum は小数でもOK
    """
    c1 = floor(carnum)
    c2 = ceil(carnum)
    if c1 == c2:
        return _getcarworldpos(trainid=trainid, carnum=c1)
    m1 = c2 - carnum
    m2 = carnum - c1
    pos1 = _getcarworldpos(trainid=trainid, carnum=c1)
    pos2 = _getcarworldpos(trainid=trainid, carnum=c2)
    return [m1*pos1[i]+m2*pos2[i] for i in range(3)] # m1,m2で線形補間


def _updateframetime(time_now):
    global _systime
    ftime = time_now - _systime
    _systime = time_now # フレームの時刻を記録
    return ftime


def _zoom(sgn, ftime):
    """ズーム
    Args:
        sgn: -1でズームイン，+1でズームアウト
        ftime: フレーム描画時間
    """
    global _fov
    _fov[0] = NXSYS.GetGlobalCameraFOV() + sgn * dFOV * ftime
    NXSYS.SetGlobalCameraFOV(_fov[0])
    _focus()


def _rotate(campos, sgn, ftime):
    """水平方向見回し
    
    Args:
        sgn: -1で左，+1で右回り
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
    """垂直見回し"""
    x = sqrt((campos[3]-campos[0])**2 + (campos[5]-campos[2])**2)
    y = campos[4]-campos[1]
    alpha = tan(sgn * dRot * ftime)
    campos[4] = (y + x*alpha)/(1 - y/x*alpha) + campos[1] # tan加法定理の変形


def _focus():
    """被写界深度を更新
    
    グローバル変数 ``_depth``, ``_fnum``, ``_fov``, ``_delta``, ``_blur``を参照して
    ボケを演算し設定します。
    """
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
    """編成リストを更新
    
    分割・併合の発生後には実行したいけど，公開プロパティにしてません。どうしよう…。
    """
    global _trainlist
    _trainlist['obj'] = []
    LAYOUT.ListTrain(_trainlist['obj'])
    _trainlist['obj'] = [t for t in _trainlist['obj'] if not t.GetDummyMode()]
    _trainlist['id'] = [t.GetID() for t in _trainlist['obj'] if not t.GetDummyMode()]
    _trainlist['name'] = [t.GetNAME() for t in _trainlist['obj'] if not t.GetDummyMode()]


def _dispgui():
    """操作パネル"""
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
    global _fuzzytrack
    global _aemode
    global _aeparam
    global _gcdist

    IMGUI.Begin("ToruoWin", "撮る夫くん")

    action = False
    action += IMGUI.SliderFloat("zoom", "FOV(ズーム角度)", _fov, 10.0, 135.0)
    action += IMGUI.SliderFloat("focus", "合焦中心", _depth, 0.1, 0.5)
    action += IMGUI.SliderFloat("fnum", "絞り(F値)", _fnum, 2.0, 200.0)
    action += IMGUI.SliderFloat("blur", "ぼけの強さ", _blur, 0.0, 2.0)
    if action:
        _focus()
    del action

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
            IMGUI.Text("撮る夫くんは保存されていません。")
        if IMGUI.Button("addtoruo", "現在視点を保存"):
            _save_toruo()
        IMGUI.SameLine()
        if IMGUI.Button("deltoruo", "現在の撮る夫くんを削除"):
            del _toruos[_childid[0]]
            _save_config()
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
            if IMGUI.Checkbox("trackfuzzy", "ファジィ追尾", _fuzzytrack):
                _tracking_carnum[0] = int(round(_tracking_carnum[0]))
            if _tracking_trainid[0]:
                if _fuzzytrack[0]:
                    if IMGUI.SliderFloat('carnofuzzy', "ファジィ号車番号", _tracking_carnum, 1.0, _tracking_trnlen):
                        _tracking_car = LAYOUT.GetTrain(_tracking_trainid[0]).GetCar(int(round(_tracking_carnum[0]))-1)
                else:
                    if IMGUI.SliderInt("carno", "号車番号", _tracking_carnum, 1, _tracking_trnlen):
                        _tracking_car = LAYOUT.GetTrain(_tracking_trainid[0]).GetCar(_tracking_carnum[0]-1)
                IMGUI.Text("車体長: {0:.1f}mm".format(vecdistance(_tracking_car.GetLinkPosition(0), _tracking_car.GetLinkPosition(1))))
            IMGUI.SliderFloat("relx", "相対X", _tracking_relative['x'], -150.0, 150.0)
            IMGUI.SliderFloat("rely", "相対Y", _tracking_relative['y'], -150.0, 150.0)
            IMGUI.SliderFloat("relz", "相対Z", _tracking_relative['z'], -150.0, 150.0)
            IMGUI.TreePop()
        IMGUI.SliderFloat("trdist", "追尾距離", _tracking_dist, 100.0, 2500.0)
        IMGUI.Text(str(_tracking_car))
        IMGUI.TreePop()

    if IMGUI.TreeNode("sunpos", "撮影環境設定"):
        IMGUI.Text("太陽位置")
        if IMGUI.SliderFloat('sun_longitude', '経度', _sunpos[0], -180.0, 180.0):
            LAYOUT.SKY().SetSunPos(_sunpos[0][0], _sunpos[1][0])
        if IMGUI.SliderFloat('sun_latiitude', '緯度', _sunpos[1], 0.0, 90.0):
            LAYOUT.SKY().SetSunPos(_sunpos[0][0], _sunpos[1][0])
        IMGUI.TreePop()        
    if IMGUI.TreeNode("details", "詳細設定"):
        IMGUI.Text("数値入力...")
        action = False
        action += IMGUI.InputFloat("directzoom", "FOV(ズーム角度)", _fov)
        action += IMGUI.InputFloat("directfocus", "合焦中心", _depth)
        action += IMGUI.InputFloat("directfnum", "絞り(F値)", _fnum)
        action += IMGUI.InputFloat("directblur", "ぼけの強さ", _blur)
        if action:
            _focus()
        del action
        
        IMGUI.Separator()

        IMGUI.Text("グローバルカメラfrom-to距離設定")
        IMGUI.InputFloat("gcdist", "設定値(mm)", _gcdist)
        if IMGUI.Button("setgcdist", "再設定"):
            set_gcdist(_gcdist[0])

        IMGUI.Text("AE詳細設定")
        IMGUI.SliderFloat('ae1', 'ぼけ限界FOV', _aeparam['blurfin'], 35.0,135.0)
        IMGUI.SliderFloat('ae2', 'F増加率', _aeparam['ftg'], 0.1,1.0)
        IMGUI.SliderFloat('ae3', 'FOV10でのF', _aeparam['f10'], 1.0, 50.0)
        IMGUI.TreePop()

    IMGUI.Separator()

    if IMGUI.Button("closer", "メニューを閉じる"):
        global _guidisp
        _guidisp = False
    if DEBUG:
        pos = NXSYS.GetGlobalCameraPos()
        pos_from = pos[:3]
        pos_at = pos[3:]
        IMGUI.Text("From: {}".format(pos_from))
        IMGUI.Text("At  : {}".format(pos_at))
        IMGUI.Text("Dist: {}".format(vecdistance(pos_from, pos_at)))
    IMGUI.End()


def vecadd(vec1, vec2):
    return [a+b for a,b in zip(vec1, vec2)]

def vecscale(k, vec):
    """ベクトルvecをスカラーk倍"""
    return list(map(lambda x: k*x, vec))


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
    