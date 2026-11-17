use std::io::Result;
use std::process::Command;

pub fn open(url: &str) -> Result<()> {
    #[cfg(target_os = "windows")]
    {
        Command::new("cmd").args(["/C", "start", "", url]).spawn()?;
        Ok(())
    }
    #[cfg(target_os = "macos")]
    {
        Command::new("open").arg(url).spawn()?;
        Ok(())
    }
    #[cfg(all(unix, not(target_os = "macos")))]
    {
        Command::new("xdg-open").arg(url).spawn()?;
        Ok(())
    }
    #[cfg(not(any(target_os = "windows", unix)))]
    {
        let _ = url;
        Ok(())
    }
}
