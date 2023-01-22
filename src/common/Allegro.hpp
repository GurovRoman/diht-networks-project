#pragma once

#include <allegro5/allegro5.h>
#include <allegro5/allegro_font.h>
#include <allegro5/allegro_primitives.h>

#include "assert.hpp"
#include "common.hpp"

template<class Derived>
class Allegro {
public:
    static constexpr int kWidth = 1280;
    static constexpr int kHeight = 720;

    Allegro()
    {
        NG_VERIFY(al_init());
        NG_VERIFY(al_install_keyboard());
        NG_VERIFY(al_install_mouse());
        NG_VERIFY(al_init_primitives_addon());
        event_queue_ = {al_create_event_queue(), &al_destroy_event_queue};
        display_ = {al_create_display(kWidth, kHeight), &al_destroy_display};
        font_ = {al_create_builtin_font(), &al_destroy_font};
        draw_timer_ = {al_create_timer(1.f / 60.f), &al_destroy_timer};

        al_register_event_source(
            event_queue_.get(), al_get_keyboard_event_source());
        al_register_event_source(event_queue_.get(), al_get_mouse_event_source());
        al_register_event_source(
            event_queue_.get(), al_get_display_event_source(display_.get()));
        al_register_event_source(
            event_queue_.get(), al_get_timer_event_source(draw_timer_.get()));

        al_start_timer(draw_timer_.get());
    }

    void poll()
    {
        bool redraw = false;

        while (!al_event_queue_is_empty(event_queue_.get())) {
            ALLEGRO_EVENT event;
            al_wait_for_event_timed(event_queue_.get(), &event, 0.f);

            switch (event.type) {
                case ALLEGRO_EVENT_TIMER:
                    if (event.timer.source == draw_timer_.get()) {
                        redraw = true;
                    }
                    break;

                case ALLEGRO_EVENT_MOUSE_AXES:
                    self().mouse(event.mouse.x, event.mouse.y);
                    break;

                case ALLEGRO_EVENT_KEY_DOWN:
                    self().keyDown(event.keyboard.keycode);
                    break;

                case ALLEGRO_EVENT_KEY_UP:
                    self().keyUp(event.keyboard.keycode);
                    break;

                case ALLEGRO_EVENT_DISPLAY_CLOSE:
                    self().close();
                    break;
            }
        }

        if (redraw) {
            al_clear_to_color(al_map_rgb(0, 0, 0));
            self().draw();
            al_flip_display();
        }
    }

    void stop()
    {
        event_queue_.reset();
        display_.reset();
    }

    ALLEGRO_FONT* getFont() { return font_.get(); }

private:
    Derived& self() { return *static_cast<Derived*>(this); }
    const Derived& self() const { return *static_cast<const Derived*>(this); }

private:
    UniquePtr<ALLEGRO_EVENT_QUEUE> event_queue_{nullptr, nullptr};
    UniquePtr<ALLEGRO_DISPLAY> display_{nullptr, nullptr};
    UniquePtr<ALLEGRO_FONT> font_{nullptr, nullptr};
    UniquePtr<ALLEGRO_TIMER> draw_timer_{nullptr, nullptr};
};
