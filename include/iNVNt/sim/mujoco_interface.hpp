#pragma once

#include <functional>

#include "GLFW/glfw3.h"
#include "mujoco/mujoco.h"
#include "iNVNt/sim/sim_data.hpp"

namespace nvn::mj
{
    /* MuJoCo Context */

    struct mjWindowContext
    {
        GLFWwindow *window = nullptr; // GLFW window
        struct
        {
            bool button_left = false;
            bool button_middle = false;
            bool button_right = false;
            double lastx = 0;
            double lasty = 0;
        } mouse;            // GLFW mouse data
        mjvCamera camera;   // Camera for rendering
        mjvPerturb perturb; // Perturbation for rendering
        mjvOption option;   // Rendering options
        mjvScene scene;     // Scene for rendering
        mjrContext context; // Rendering context
    };

    struct mjSimContext
    {
        mjModel *model = nullptr; // MuJoCo model
        mjData *data = nullptr;   // MuJoCo data
    };

    inline auto &_mjWindowContext()
    {
        static mjWindowContext data; // Global GLFW data
        return data;
    }

    inline auto &_mjSimContext()
    {
        static mjSimContext data; // Global MuJoCo data
        return data;
    }

    /* Keyboard/Mouse Callbacks */

    static inline void _glfwKeyPressCallback(GLFWwindow *, int key, int, int act, int)
    {
        auto &sc = _mjSimContext();
        if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
        {
            mj_resetData(sc.model, sc.data);
            mj_forward(sc.model, sc.data);
        }
    }

    static inline void _glfwMouseClickCallback(GLFWwindow *window, int, int, int)
    {
        auto &wc = _mjWindowContext();
        wc.mouse.button_left = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        wc.mouse.button_middle = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
        wc.mouse.button_right = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
        glfwGetCursorPos(window, &wc.mouse.lastx, &wc.mouse.lasty);
    }

    static inline void _glfwMouseMoveCallback(GLFWwindow *window, double xpos, double ypos)
    {
        auto &wc = _mjWindowContext();
        auto &sc = _mjSimContext();
        if (!wc.mouse.button_left && !wc.mouse.button_middle && !wc.mouse.button_right)
            return;

        const double dx = xpos - wc.mouse.lastx;
        const double dy = ypos - wc.mouse.lasty;
        wc.mouse.lastx = xpos;
        wc.mouse.lasty = ypos;

        int width, height;
        glfwGetWindowSize(window, &width, &height);

        bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) ||
                         (glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

        mjtMouse action;
        if (wc.mouse.button_right)
        {
            // camera movement
            action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
        }
        else if (wc.mouse.button_left)
        {
            // camera rotation
            action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
        }
        else
        {
            // camera zoom
            action = mjMOUSE_ZOOM;
        }

        mjv_moveCamera(sc.model, action, dx / height, dy / height, &wc.scene, &wc.camera);
    }

    static inline void _glfwMouseScrollCallback(GLFWwindow *, double, double yoffset)
    {
        auto &wc = _mjWindowContext();
        auto &sc = _mjSimContext();
        mjv_moveCamera(sc.model, mjMOUSE_ZOOM, 0, 0.05 * yoffset, &wc.scene, &wc.camera);
    }

    /* Control Callback */

    inline auto &mjSimData()
    {
        static SimData sd(_mjSimContext().model ? _mjSimContext().model->nu : 0);
        return sd;
    }

    inline void _mjControlCallback(const mjModel *m, mjData *d)
    {
        auto &sd = mjSimData();
        sd.time = d->time;
        const mjtNum *qpos = d->qpos;
        sd.position = {qpos[0], qpos[1], qpos[2]};
        sd.orientation = {qpos[3], qpos[4], qpos[5], qpos[6]};

        const mjtNum *qvel = d->qvel;
        sd.linear_velocity = {qvel[0], qvel[1], qvel[2]};
        sd.angular_velocity = {qvel[3], qvel[4], qvel[5]};

        sd.joint_positions.resize(m->nu);
        sd.joint_velocities.resize(m->nu);
        for (std::size_t i = 0; i < sd.joint_positions.size(); i++)
        {
            const int joint_id = m->actuator_trnid[i * 2];
            const int qpos_adr = m->jnt_qposadr[joint_id];
            const int qvel_adr = m->jnt_dofadr[joint_id];
            sd.joint_positions[i] = d->qpos[qpos_adr];
            sd.joint_velocities[i] = d->qvel[qvel_adr];
        }

        sd.joint_torques.resize(m->nu);
        for (std::size_t i = 0; i < sd.joint_torques.size(); i++)
        {
            d->ctrl[i] = sd.joint_torques[i];
        }
    }

    inline bool mjLoadModel(std::string_view model_path)
    {
        auto &wc = _mjWindowContext();
        auto &sc = _mjSimContext();
        char error[1000]; // ignore for now
        sc.model = mj_loadXML(model_path.data(), nullptr, error, 1000);
        if (!sc.model)
        {
            return false;
        }
        sc.data = mj_makeData(sc.model);
        if (!sc.data)
        {
            mj_deleteModel(sc.model);
            sc.model = nullptr;
            return false;
        }
        mjv_defaultCamera(&wc.camera);
        mjv_defaultPerturb(&wc.perturb);
        mjv_defaultOption(&wc.option);
        mjr_defaultContext(&wc.context);
        mjv_makeScene(sc.model, &wc.scene, 1000);
        mjcb_control = _mjControlCallback;
        return true;
    }

    inline void mjFreeModel()
    {
        auto &sc = _mjSimContext();
        if (sc.data)
            mj_deleteData(sc.data);
        if (sc.model)
            mj_deleteModel(sc.model);
    }

    inline bool mjOpenWindow(int width = 1200, int height = 900, std::string_view name = "MuJoCo")
    {
        auto &wc = _mjWindowContext();
        auto &sc = _mjSimContext();
        if (!glfwInit())
        {
            return false;
        }
        wc.window = glfwCreateWindow(width, height, name.data(), nullptr, nullptr);
        if (!wc.window)
        {
            glfwTerminate();
            return false;
        }
        glfwMakeContextCurrent(wc.window);
        glfwSwapInterval(1);
        glfwSetKeyCallback(wc.window, _glfwKeyPressCallback);
        glfwSetCursorPosCallback(wc.window, _glfwMouseMoveCallback);
        glfwSetMouseButtonCallback(wc.window, _glfwMouseClickCallback);
        glfwSetScrollCallback(wc.window, _glfwMouseScrollCallback);
        mjr_makeContext(sc.model, &wc.context, mjFONTSCALE_150);
        return true;
    }

    inline bool mjShouldWindowClose()
    {
        auto &wc = _mjWindowContext();
        if (!wc.window)
            return false;
        return glfwWindowShouldClose(wc.window);
    }

    inline void mjCloseWindow()
    {
        auto &wc = _mjWindowContext();
        if (wc.window)
        {
            glfwDestroyWindow(wc.window);
            wc.window = nullptr;
            glfwTerminate();
        }
        mjv_freeScene(&wc.scene);
        mjr_freeContext(&wc.context);
    }

    inline void mjStep(const mjtNum dt)
    {
        auto &sc = _mjSimContext();
        const mjtNum simend = sc.data->time + dt;
        while (sc.data->time < simend)
        {
            mj_step(sc.model, sc.data);
        }
    }

    inline void mjRender()
    {
        auto &wc = _mjWindowContext();
        auto &sc = _mjSimContext();
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(wc.window, &viewport.width, &viewport.height);

        mjv_updateScene(sc.model, sc.data, &wc.option,
                        &wc.perturb, &wc.camera, mjCAT_ALL, &wc.scene);
        mjr_render(viewport, &wc.scene, &wc.context);

        glfwSwapBuffers(wc.window);
        glfwPollEvents();
    }
}