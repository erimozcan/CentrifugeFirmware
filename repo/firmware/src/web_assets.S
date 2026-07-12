/* Embedded web UI assets for the WiFi console. Each file is linked into flash via
 * .incbin (paths are relative to the PlatformIO project dir, repo/firmware). The
 * net server serves these directly from flash. Regenerate data/index.html.gz from
 * repo/ui/index.html when the UI changes (see the build note in net_server.cpp).
 * Only linked into the WiFi build (WITH_WEB_ASSETS) -- ~1 MB would overflow the Due. */
#ifdef WITH_WEB_ASSETS
  .section .rodata.webassets
  .balign 4

  .global webasset_index_gz_start
webasset_index_gz_start:
  .incbin "data/index.html.gz"
  .global webasset_index_gz_end
webasset_index_gz_end:

  .balign 4
  .global webasset_poster_start
webasset_poster_start:
  .incbin "data/poster.jpg"
  .global webasset_poster_end
webasset_poster_end:

  .balign 4
  .global webasset_idle_start
webasset_idle_start:
  .incbin "data/clips/idle.mp4"
  .global webasset_idle_end
webasset_idle_end:

  .balign 4
  .global webasset_opening_start
webasset_opening_start:
  .incbin "data/clips/opening.mp4"
  .global webasset_opening_end
webasset_opening_end:

  .balign 4
  .global webasset_open_start
webasset_open_start:
  .incbin "data/clips/open.mp4"
  .global webasset_open_end
webasset_open_end:

  .balign 4
  .global webasset_spin_start
webasset_spin_start:
  .incbin "data/clips/spin.mp4"
  .global webasset_spin_end
webasset_spin_end:

  .balign 4
  .global webasset_closing_start
webasset_closing_start:
  .incbin "data/clips/closing.mp4"
  .global webasset_closing_end
webasset_closing_end:

  .balign 4
  .global webasset_logo_gz_start
webasset_logo_gz_start:
  .incbin "data/logo.svg.gz"
  .global webasset_logo_gz_end
webasset_logo_gz_end:

#endif  /* WITH_WEB_ASSETS */
