import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

export default defineConfig({
  plugins: [react()],
  server: {
    port: 5173,
    // Dev client uses apiOrigin() → http://127.0.0.1:8000 (see App.tsx), so no proxy needed.
  },
});
