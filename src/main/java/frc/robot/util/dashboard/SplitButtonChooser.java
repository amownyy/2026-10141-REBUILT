package frc.robot.util.dashboard;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Collectors;

public class SplitButtonChooser<T> implements Sendable {
    private final Function<String, T> stringToType;
    private final Function<T, String> typeToString;
    private final Supplier<T> active;
    private final Collection<String> options;
    private final Set<Consumer<T>> listeners;
    private String defaultOption;
    private String selected;

    public SplitButtonChooser(
            final Supplier<T> active,
            final Collection<T> options,
            final Set<Consumer<T>> listeners,
            final T defaultOption,
            final Function<String, T> stringToType,
            final Function<T, String> typeToString) {
        this.active = Objects.requireNonNullElse(active, this::getSelected);
        this.options = Objects.requireNonNullElse(options, new ArrayList<T>()).stream()
                .map(typeToString)
                .collect(Collectors.toList());
        this.listeners = Objects.requireNonNullElse(listeners, Collections.newSetFromMap(new ConcurrentHashMap<>()));
        this.defaultOption = defaultOption == null ? null : typeToString.apply(defaultOption);
        this.stringToType = stringToType;
        this.typeToString = typeToString;
    }

    public static SplitButtonChooser<String> withStrings(
            final Supplier<String> active,
            final Collection<String> options,
            final Set<Consumer<String>> listeners,
            final String defaultOption) {
        return new SplitButtonChooser<>(
                active, options, listeners, defaultOption, Function.identity(), Function.identity());
    }

    public void addOption(final T option) {
        final String string = typeToString.apply(option);
        if (!options.contains(string)) {
            options.add(string);
        }
    }

    public void setDefaultOption(final T defaultOption) {
        this.defaultOption = typeToString.apply(defaultOption);
        addOption(defaultOption);
    }

    public T getSelected() {
        if (selected != null) {
            return stringToType.apply(selected);
        }
        return defaultOption == null ? null : stringToType.apply(defaultOption);
    }

    public void onChange(final Consumer<T> listener) {
        listeners.add(listener);
    }

    @Override
    public void initSendable(final SendableBuilder builder) {
        builder.setSmartDashboardType("Split Button Chooser");
        builder.addStringProperty("default", () -> Objects.requireNonNullElse(defaultOption, ""), null);
        builder.addStringArrayProperty("options", () -> options.toArray(new String[0]), null);
        builder.addStringProperty(
                "active",
                () -> {
                    final T activeValue = active.get();
                    return activeValue == null ? null : typeToString.apply(activeValue);
                },
                null);
        builder.addStringProperty("selected", null, newValue -> {
            final String before = selected;
            selected = newValue;
            if (!Objects.equals(before, newValue)) {
                listeners.forEach(listener -> listener.accept(stringToType.apply(newValue)));
            }
        });
    }
}
