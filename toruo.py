# -*- coding: utf-8 -*-

# MIT License
#
# Copyright (c) 2021-2025 AKAGI-Rails
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""撮る夫くん - VRMNX用グローバルカメラ拡張機能

VRMNX用Python拡張の **撮る夫くん** は，VRMNXビュワーのフライスルーカメラの機能をアップグレードします。
ImGUIの操作パネルで，FOVや被写界深度の設定を直感的に行うことができます。

撮る夫くんには以下のような機能があります。

- FOVなどのGUI操作
- FOV, F値と対象物までの距離から，一眼レフカメラの機構をシミュレートした被写界深度制御
- 手ブレ風エフェクト
- 車両追尾
- 視点保存
- ゲームパッドでの操作

Example:
    撮る夫くんを有効にするには，レイアウトのイベントハンドラの冒頭に
    ``toruo.activate()`` を記述します::
    
        import vrmnx
        import toruo
        
        def vrmevent(obj,ev,param):
            toruo.activate(obj,ev,param)
            if ev == 'init':
                pass
    
イベントのuserIDの予約領域

撮る夫くんでは，VRMNXのイベントuserIDで以下の領域を予約します。::

    1060000 - 1069999

撮る夫くんが内部で使用するイベントはこの領域内でuserIDを指定しています。
"""

__all__ = ['DEBUG', 'dFOV', 'dRot', 'dMov', 'shake_factor', 'shake_freq',
           'activate', 'set_toruo', 'jump_toruo', 'setfactor', 'setshakemode', 'set_gcdist',
           'screenshot']
__version__ = '3.4.0'
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
import ctypes
import datetime
from itertools import chain

from mss.windows import MSS as msswin
import mss.tools
import pygetwindow as gw

DEBUG = True

LAYOUT = vrmapi.LAYOUT()
NXSYS = vrmapi.SYSTEM()
IMGUI = vrmapi.ImGui()
DIRECTORY, TAIL = os.path.split(NXSYS.GetLayoutPath())
#LOG(TAIL)

# フレームイベントの設定は activate イベントハンドラでinitのタイミングに移動

_PARENT = None  # 親オブジェクト
#_gcam = []     # 地上カメラのリスト
_config = None  # 撮る夫くん設定データ
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
DEFAULT_AEPARAM = {'blurfin':[90.0], 'ftg':[5.0/12], 'f10':[25.0]}
_aeparam = DEFAULT_AEPARAM
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

# ゲームパッドFLG
_GPlist = [False, False, False, False]  #: 接続されているとTrue 
_gamepad_sw = [-1]  #: -1で無効、0-3で撮る夫くんで使用するゲームパッドのデバイス番号
DEFAULT_GAMEPAD_PARAM = {'L0_sense':[1.0], 'L0_exp':[1.0], 'R0_sense':[1.0], 'R0_exp':[1.0], 'R0_Yinv':[False], 'v_sense':[1.0], 'zoom_sense':[1.0]}
_gamepad_param = DEFAULT_GAMEPAD_PARAM
_gamepad_RYsgn = 1
_dash_factor = 1.0  #: ダッシュ係数（1.0～2.0）
DASH_MAX = 3.0
DASH_DIFF = 8.0

ANALOG_MAX = 32768.0

GP_BUTTONS = [1,2,4,8, 0x10, 0x20, 0x100, 0x200, 0x1000, 0x2000, 0x4000, 0x8000]

# Event UserID 1060000 - 1069999
EVUID_TORUOFRAME = 1060000
EVUID_TORUOSWITCH = 1060001
EVUID_TORUOSHAKE = 1060101


# マイピクチャを取得（スクリーンショットの保存先）
_buf = ctypes.create_unicode_buffer(ctypes.wintypes.MAX_PATH)
if ctypes.windll.shell32.SHGetSpecialFolderPathW(None, _buf, 0x0027, False):
    MYPICTURES = _buf.value
else:
    vrmapi.LOG('[toruo] Failed to get Pictures folder path.')
    MYPICTURES = os.getcwd()
del _buf

SSSAVEDIR = os.path.join(MYPICTURES, "screenshot")
os.makedirs(SSSAVEDIR, exist_ok=True)

APPNAMES = ('鉄道模型シミュレーターNX', 'VRMONLINE-NX')

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
        refresh_GPlist()
        _load_config()
        _refresh_trainlist()
        vrmapi.LOG('撮る夫くん(Ver.{}) stand by. {}'.format(__version__, DIRECTORY))
        #print(MYPICTURES)
        return

    elif param['eventid'] == _shake_evid:
        # (Afterイベント)
        _update_shake()
        return
    
    elif ev == 'keydown':
        if param['keycode'] == 'P':
            # GUI表示のON/OFFを切替
            # このイベントは他モジュールのイベントとの相互乗り入れのため、IDで区別しない
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

    global _shake_hr
    global _shake_vt
    global _dash_factor
    global _depth

    """
    # キー操作の処理
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
    """
    # ゲームパッド操作
    if _gamepad_sw[0] in [0,1,2,3]:
        gp = _gamepad_sw[0]

        # スクリーンショット(mss)
        if NXSYS.GetGamepadB(gp):
            screenshot()

        # ダッシュ処理
        if NXSYS.GetGamepadA(gp):
            _dash_factor += DASH_DIFF * ftime
        else:
            _dash_factor -= DASH_DIFF * ftime
        _dash_factor = clip(_dash_factor, 1.0, DASH_MAX)

        # 並進移動
        # 左スティック・水平面
        LX = NXSYS.GetGamepadAnalogStickLX(gp)
        LY = NXSYS.GetGamepadAnalogStickLY(gp)
        # 上下移動
        v = 0
        if NXSYS.GetGamepadRB(gp):
            v += +1 * _gamepad_param['v_sense'][0]
        if NXSYS.GetGamepadLB(gp):
            v += -1 * _gamepad_param['v_sense'][0]
        if abs(LX) > 100 or abs(LY) > 100 or v != 0:
            _move(campos, adjust_analogL(LX)*_dash_factor, v*_dash_factor, adjust_analogL(LY)*_dash_factor, ftime)
        else:
            if not NXSYS.GetGamepadA(gp):
                _dash_factor = 1.0

        # 見回し（ゲームパッド）
        RX = NXSYS.GetGamepadAnalogStickRX(gp)
        if abs(RX) > 100:
            _rotate(campos, adjust_analogR(RX), ftime)

        RY = NXSYS.GetGamepadAnalogStickRY(gp)
        if abs(RY) > 100:
            _rotatevt(campos, adjust_analogR(RY)*_gamepad_RYsgn, ftime)

        # ズームイン・ズームアウト（ゲームパッド）
        stat = NXSYS.GetGamepadLEFT(gp)
        if stat:
            _zoom(1*_gamepad_param['zoom_sense'][0], ftime)
        
        stat = NXSYS.GetGamepadRIGHT(gp)
        if stat:
            _zoom(-1*_gamepad_param['zoom_sense'][0], ftime)

        # フォーカス（深度値）中心
        if NXSYS.GetGamepadUP(gp):
            # フォーカス遠方へ＝深度値小さく
            d = _depth[0] - 0.05*ftime
            _depth[0] = clip(d, 0.1, 0.5)
            _focus()
        if NXSYS.GetGamepadDOWN(gp):
            # フォーカス近くへ＝深度値大きく
            d = _depth[0] + 0.05*ftime
            _depth[0] = clip(d, 0.1, 0.5)
            _focus()

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
    
    保存済みの撮る夫くんを ``id`` で呼び出し，その座標にジャンプします。
    
    Args:
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

    たとえば， initイベントで実行すると初期状態の撮る夫くんの設定ができます。 ::

        # LAYOUT
        import vrmapi
        import toruo

        def vrmevent(obj,ev,param):
            toruo.activate(obj,ev,param)
            if ev == 'init':
                toruo.set_toruo(depth=512**(-1/4), fnum=9.0)

    カメラ座標は ``vrmapi.SYSTEM().SetGlobalCameraPos()`` によってください。
        
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

    Parameters:
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


def refresh_GPlist():
    """接続されているゲームパッドのリストを更新"""
    global _GPlist
    gplist = [NXSYS.IsGamepadConnected(i) for i in range(4)]
    _GPlist = gplist
    vrmapi.LOG("[toruo] Gamepad connection refreshed.")
    return gplist


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


def _default_config():
    """空の設定ファイルを返す
    
    撮る夫くんconfig v.3.0
    """
    return {
        'toruoconfig': 'toruoconfig',  # ファイル書式宣言。これがないjsonは相手にしない。
        'version': '3.0',  # 書式バージョン宣言

        'layouts': {
            # dict of dict - レイアウトごとの情報を保存。順不同。
            # key: レイアウトのファイルパスのTAIL (ファイル名)
            # value: 設定内容(dict)

            # TAIL: {
            #     'filename': TAIL,  # ビュワーが開いているファイルとこの値で一致判定する。
            #     'timestamp': str(datetime.datetime.now()),
            # 
            #     # 'toruo'要素は旧バージョンと同等のリスト。
            #     # 保存済み撮る夫くんのID-1がリストの番号になる。
            #     'toruos': _toruos
            #     
            #     'gamepad': (int) #: ゲームパッドスイッチの状態（撮る夫くんでアクティブなデバイス番号。-1はOFF）
            #     'gamepad_param': {'L0_sense':[1.0], 'L0_exp':[1.0], 'R0_sense':[1.0], 'R0_exp':[1.0], 'v_sense':[1.0]} by default
            #     'aeparam': (dict) AEパラメータ
            # }
        }
    }


def _check_config(config):
    """撮る夫くん設定ファイルのチェック
    
    正規データに対しては何もせずそのまま返す。
    無効なデータは，空の正規データで返す。

    撮る夫くんconfig - v.3.0
    """
    # マジックヘッダの確認
    try:
        magic = config['toruoconfig']
    except (TypeError, KeyError) as e:
        LOG("Invalid toruo setting")
        print("Following error is muted;", e)
        return _default_config()

    if magic != 'toruoconfig':
        LOG("[Toruo Warning] Invalid magicheader for toruo setting.")
        return _default_config()
    
    # バージョンのチェック (New in toruo v.3.4)
    v = config.get('version')
    if int(v[0]) < 3:
        return _default_config()

    return config


def _load_config(filename=os.path.join(DIRECTORY, 'toruo.json')):
    """保存済み撮る夫くんを読み込み

    撮る夫くんconfig - v.3.0
    
    レイアウトと同じディレクトリの toruo.json を探し，存在すればデータを読み込みます。
    """
    global _config
    global _toruos
    global _gamepad_sw
    global _gamepad_param
    global _aeparam
    try:
        with open(filename) as js:
            config = json.load(js)
    except FileNotFoundError:
        LOG("[Toruo Warning] Could not found toruo setting.")
        _config = _default_config()
        return

    config = _check_config(config)
    _config = config

    # 自分のレイアウトファイルに相当するデータの引っ張り出し
    lay = config['layouts'].get(TAIL, None)
    if lay is None:
        # 自分のレイアウトのセーブデータがない
        LOG("[Toruo Warning] toruo data is not found.")
    else:
        # セーブデータが見つかった
        _toruos = lay.get('toruos', [])  # 撮る夫くんのリストをロード

        # ゲームパッド感度設定をロードして復活
        _gamepad_param = lay.get('gamepad_param', {})
        for k,v in DEFAULT_GAMEPAD_PARAM.items():
            _gamepad_param.setdefault(k,v)
        global _gamepad_RYsgn
        if _gamepad_param['R0_Yinv'][0]:
            _gamepad_RYsgn = -1
        else:
            _gamepad_RYsgn = 1

        # プログラムAE
        _aeparam = lay.get('aeparam', {})
        for k,v in DEFAULT_AEPARAM.items():
            _aeparam.setdefault(k,v)
        
        # ゲームパッドの設定をロードして復活
        gp = lay.get('gamepad', -1)
        if gp in [0,1,2,3]:
            if NXSYS.IsGamepadConnected(gp):
                # ゲームパッドの接続あり
                _gamepad_sw = [gp]
            else:
                _gamepad_sw = [-1]
        else:
            _gamepad_sw = [-1]
        _change_gamepad()  # これは最後に（saveが呼ばれちゃうので）


def _save_config(filename=os.path.join(DIRECTORY, 'toruo.json')):
    """撮る夫くん保存状況をファイルに書き出し
    
    撮る夫くんconfig - v.3.0
    """
    global _config
    config = _check_config(_config)

    config['layouts'][TAIL] = {
            'filename': TAIL,  # ビュワーが開いているファイルとこの値で一致判定する。
            'timestamp': str(datetime.datetime.now()),

            # 'toruo'要素は旧バージョンと同等のリスト。
            # 保存済み撮る夫くんのID-1がリストの番号になる。
            'toruos': _toruos,

            # ゲームパッドスイッチの状態。（撮る夫くんでアクティブなデバイス番号。-1はOFF）
            'gamepad': _gamepad_sw[0],
            'gamepad_param': _gamepad_param,

            'aeparam': _aeparam
        }
    _config = config  # グローバルに書き戻す
    with open(filename, 'w') as js:
        json.dump(config, js, indent=4)
    vrmapi.LOG("toruo config saved.")
    return


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


def _zoom(spd, ftime):
    """ズーム
    Args:
        spd: -1でズームイン，+1でズームアウト
        ftime: フレーム描画時間
    """
    global _fov
    f = NXSYS.GetGlobalCameraFOV() + spd * dFOV * ftime
    f = clip(f, 10.0, 135.0)
    _fov[0] = f
    NXSYS.SetGlobalCameraFOV(_fov[0])
    _focus()


def _move(campos, x, y, z, ftime):
    """平行移動

    リストで与えられる `campos` をインプレースに操作します。
    帰り値はありません。
    
    Args:
        campos: NXSYS.GetGlobalCameraPos()の帰り値と同型
        xyz: 入力移動量。camposと同じ次元。
        ftime: フレームの描画時間

    入力移動量のXZは向いてる方角に回転して現在座標に加算する。

    Note:
        VRM Layoutのワールド座標XZとスティック入力のXYで回転方向が違うので注意。
    """
    # 0 1 2 3 4 5
    # x y z x y z
    # XZ方向を回転
    rx = campos[3] - campos[0]
    rz = campos[5] - campos[2]
    vr = [rx, rz]
    vl = veclen(vr)
    cos_, sin_ = vecscale(1/vl/100, vr)  # [cos, sin]

    rot = [[-sin_, 0, cos_],
           [0, 100, 0],
           [cos_, 0, sin_]]
    vm = vectrans(rot, [x,y,z])  # 回転済みの入力方向
    vm = vecscale(ftime, vm)
    campos[0] += vm[0]
    campos[1] += vm[1]
    campos[2] += vm[2]
    campos[3] += vm[0]
    campos[4] += vm[1]
    campos[5] += vm[2]


def adjust_analogL(x):
    sgnx = sgn(x)
    absx = abs(x)
    y = sgnx * ((absx/ANALOG_MAX)**_gamepad_param['L0_exp'][0]) * _gamepad_param['L0_sense'][0] * ANALOG_MAX
    return y

def _rotate(campos, spd, ftime):
    """水平方向見回し

    リストで与えられる `campos` をインプレースに操作します。
    帰り値はありません。
    
    Args:
        campos: NXSYS.GetGlobalCameraPos()の帰り値と同型
        spd: -1で左，+1で右回り
        ftime: フレームの描画時間
    """
    # list campos はMutableなので返り値を取らなくていい
    dx = campos[3] - campos[0]
    dz = campos[5] - campos[2]
    dtheta = spd * dRot * ftime
    cos_ = cos(dtheta)
    sin_ = sin(dtheta)
    campos[3] = campos[0] + cos_*dx - sin_*dz
    campos[5] = campos[2] + sin_*dx + cos_*dz


def _rotatevt(campos, spd, ftime):
    """垂直見回し

    リストで与えられる `campos` をインプレースに操作します。
    帰り値はありません。
    
    Args:
        campos: NXSYS.GetGlobalCameraPos()の帰り値と同型
        spd: -1で下，+1で上
        ftime: フレームの描画時間
    """
    x = sqrt((campos[3]-campos[0])**2 + (campos[5]-campos[2])**2)
    y = campos[4]-campos[1]
    alpha = tan(spd * dRot * ftime)
    campos[4] = (y + x*alpha)/(1 - y/x*alpha) + campos[1] # tan加法定理の変形

def adjust_analogR(x):
    sgnx = sgn(x)
    absx = abs(x)
    y = sgnx * ((absx/ANALOG_MAX)**_gamepad_param['R0_exp'][0]) * _gamepad_param['R0_sense'][0] 
    return y

def _focus():
    """被写界深度を更新
    
    グローバル変数 `_depth`, `_fnum`, `_fov`, `_delta`, `_blur` を参照して
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
    global _gamepad_sw

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
            IMGUI.TreePop()
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
        IMGUI.Text("ゲームパッド感度調整")
        if IMGUI.InputFloat("L0Sense", "左スティック・倍率", _gamepad_param['L0_sense']):
            _gamepad_param['L0_sense'][0] = clip(_gamepad_param['L0_sense'][0], -1000.0, 1000.0)
            _save_config()
        if IMGUI.InputFloat("L0exp", "左スティック・低速", _gamepad_param['L0_exp']):
            _gamepad_param['L0_exp'][0] = clip(_gamepad_param['L0_exp'][0], 0.0001, 1000.0)
            _save_config()
        if IMGUI.InputFloat("R0Sense", "右スティック・倍率", _gamepad_param['R0_sense']):
            _gamepad_param['R0_sense'][0] = clip(_gamepad_param['R0_sense'][0], -1000.0, 1000.0)
            _save_config()
        if IMGUI.InputFloat("R0exp", "右スティック・低速", _gamepad_param['R0_exp']):
            _gamepad_param['R0_exp'][0] = clip(_gamepad_param['R0_exp'][0], 0.0001, 1000)
            _save_config()
        if IMGUI.Checkbox('RYinv', "右スティック・上下反転", _gamepad_param['R0_Yinv']):
            global _gamepad_RYsgn
            if _gamepad_param['R0_Yinv'][0]:
                _gamepad_RYsgn = -1
            else:
                _gamepad_RYsgn = 1
        if IMGUI.InputFloat("vsense", "上下移動", _gamepad_param['v_sense']):
            _gamepad_param['v_sense'][0] = clip(_gamepad_param['v_sense'][0], -1000.0, 1000.0)
            _save_config()
        if IMGUI.InputFloat("zoomsense", "ズーム", _gamepad_param['zoom_sense']):
            _gamepad_param['zoom_sense'][0] = clip(_gamepad_param['zoom_sense'][0], -1000.0, 1000.0)
            _save_config()

        IMGUI.Separator()

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

        IMGUI.Separator()

        IMGUI.Text("AE詳細設定")
        IMGUI.SliderFloat('ae1', 'ぼけ限界FOV', _aeparam['blurfin'], 20.0,135.0)
        IMGUI.SliderFloat('ae2', 'F増加率', _aeparam['ftg'], 0.1,1.0)
        IMGUI.SliderFloat('ae3', 'FOV10でのF', _aeparam['f10'], 1.0, 50.0)

        IMGUI.Separator()

        if IMGUI.Button('save_details', "詳細設定を保存"):
            _save_config()

        IMGUI.TreePop()

    IMGUI.Separator()

    IMGUI.Text("ゲームパッドで撮る夫くん操作")

    if IMGUI.RadioButton("GPnull", "ゲームパッドOFF", _gamepad_sw, -1):
        _change_gamepad()
        # LAYOUT.DispTickerMSG("[撮る夫くん]ゲームパッド OFF")

    #for i in range(4):
    #    if NXSYS.IsGamepadConnected(i):
    for i, con in enumerate(_GPlist):
        if con:
            if IMGUI.RadioButton(f'GP{i}', f"ゲームパッド{i}", _gamepad_sw, i):
                _change_gamepad()
                # LAYOUT.DispTickerMSG(f"[撮る夫くん]ゲームパッド = {i}")

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
        IMGUI.Text("L   : {}, {}".format(NXSYS.GetGamepadAnalogStickLX(0), NXSYS.GetGamepadAnalogStickLY(0)))
        IMGUI.Text("Dash: {}".format(_dash_factor))
    IMGUI.End()


def _change_gamepad():
    """ゲームパッドのアクティブ状態を更新。
    
    グローバル変数 `_gamepad_sw` の状態により、
    ゲームパッド0-4のアクティブ状態を更新します。
    """
    for i in range(4):
        if i == _gamepad_sw[0]:
            # 撮る夫くんで占有
            for k in GP_BUTTONS:
                NXSYS.SetGamepadButtonEnable(i, k, True)
        else:
            # 撮る夫くんで占有しない
            # システムのデフォルト操作に戻す
            for k in GP_BUTTONS:
                NXSYS.SetGamepadButtonEnable(i, k, False)
    _save_config()


def screenshot():
    """スクリーンショットを撮影"""
    # 保存パス
    now = datetime.datetime.now()
    sssavepath = os.path.join(SSSAVEDIR, "toruo_{}.png".format(now.strftime("%Y%m%d-%H%M%S_%f")))

    # 撮影領域の特定
    windows = list(chain.from_iterable([gw.getWindowsWithTitle(t) for t in APPNAMES]))
    if not windows:
        LOG('[toruo warning] VRM window not found.')
        return
    window = windows[0]

    with msswin() as sct:
        region = {'top':window.top, 
                    'left':window.left, 
                    'width':window.width, 
                    'height':window.height,
                }
        screenshot = sct.grab(region)
        mss.tools.to_png(screenshot.rgb, screenshot.size, output=sssavepath)
    LOG("[toruo] screenshot saved on {}".format(sssavepath))


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


def sgn(x):
    """Get sign of variable x."""
    return (x > 0) - (x < 0)

def clip(x, lb=None, ub=None):
    """Clip (limit) the value of x between [lb, ub]"""
    if ub < lb:
        raise ValueError(f"Invalid bounds are given; lb={lb}, ub={ub}")
    if lb is not None:
        if x < lb:
            return lb
    if ub is not None:
        if x > ub:
            return ub
    return x

if __name__== "__main__":
    print('toruoはVRMNXシステムでのみ有効なモジュールです')
    